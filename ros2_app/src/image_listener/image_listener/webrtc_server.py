import asyncio
import inspect
import json
import os
from typing import Any, Callable

from aiohttp import web
import aiohttp_cors
from aiortc import (
    RTCConfiguration,
    RTCIceServer,
    RTCPeerConnection,
    RTCSessionDescription,
)
from aiortc.contrib.media import MediaRelay


DEFAULT_TURN_URLS = (
    "turn:10.68.0.133:3478?transport=udp",
    "turn:10.68.0.133:3478?transport=tcp",
)


def _csv_values(value: str) -> list[str]:
    return [item.strip() for item in value.split(",") if item.strip()]


def ice_config_from_env() -> RTCConfiguration:
    if "WEBRTC_TURN_URLS" in os.environ:
        turn_urls = _csv_values(os.environ["WEBRTC_TURN_URLS"])
    elif "WEBRTC_TURN_URL" in os.environ:
        turn_urls = _csv_values(os.environ["WEBRTC_TURN_URL"])
    else:
        turn_urls = list(DEFAULT_TURN_URLS)

    if not turn_urls:
        return RTCConfiguration(iceServers=[])

    return RTCConfiguration(
        iceServers=[
            RTCIceServer(
                urls=turn_urls,
                username=os.environ.get("WEBRTC_TURN_USER", "dummy"),
                credential=os.environ.get("WEBRTC_TURN_PASSWORD", "dummy"),
            )
        ]
    )


DEFAULT_ICE_CONFIG = ice_config_from_env()


def public_turn_urls_from_env() -> list[str]:
    if "WEBRTC_PUBLIC_TURN_URLS" in os.environ:
        return _csv_values(os.environ["WEBRTC_PUBLIC_TURN_URLS"])
    if "WEBRTC_PUBLIC_TURN_URL" in os.environ:
        return _csv_values(os.environ["WEBRTC_PUBLIC_TURN_URL"])
    if "WEBRTC_TURN_URLS" in os.environ:
        return _csv_values(os.environ["WEBRTC_TURN_URLS"])
    if "WEBRTC_TURN_URL" in os.environ:
        return _csv_values(os.environ["WEBRTC_TURN_URL"])
    return list(DEFAULT_TURN_URLS)


def _ice_server_dict(urls: list[str]) -> dict[str, Any]:
    return {
        "urls": urls,
        "username": os.environ.get("WEBRTC_TURN_USER", "dummy"),
        "credential": os.environ.get("WEBRTC_TURN_PASSWORD", "dummy"),
    }


def _ice_config_urls(ice_config: RTCConfiguration) -> list[str]:
    urls: list[str] = []
    for server in ice_config.iceServers or []:
        if isinstance(server.urls, str):
            urls.append(server.urls)
        else:
            urls.extend(server.urls)
    return urls


def _candidate_summary(sdp: str | None) -> str:
    counts: dict[str, int] = {}
    for line in (sdp or "").splitlines():
        if not line.startswith("a=candidate:"):
            continue
        parts = line.split()
        if "typ" in parts:
            candidate_type = parts[parts.index("typ") + 1]
        else:
            candidate_type = "unknown"
        counts[candidate_type] = counts.get(candidate_type, 0) + 1

    if not counts:
        return "none"

    return ", ".join(f"{key}={value}" for key, value in sorted(counts.items()))


PeerSetupCallback = Callable[[RTCPeerConnection, dict[str, Any]], Any]
MessageCallback = Callable[[str | bytes], Any]


class WebRTCServer:
    """HTTP signaling wrapper that knows nothing about ROS publishers/subscribers."""

    def __init__(
        self,
        host: str = "0.0.0.0",
        port: int = 8088,
        ice_config: RTCConfiguration = DEFAULT_ICE_CONFIG,
        ice_gathering_timeout: float = 10.0,
    ):
        self.host = host
        self.port = port
        self._ice_config = ice_config
        self._ice_gathering_timeout = ice_gathering_timeout
        self._pcs: set[RTCPeerConnection] = set()
        self._media_relay = MediaRelay()
        self._app = web.Application()
        self._cors = aiohttp_cors.setup(
            self._app,
            defaults={
                "*": aiohttp_cors.ResourceOptions(
                    allow_credentials=True,
                    expose_headers="*",
                    allow_headers="*",
                )
            },
        )
        self._app.on_shutdown.append(self._on_shutdown)
        self._add_config_routes()
        print(
            "WebRTC server ICE URLs: "
            f"{', '.join(_ice_config_urls(self._ice_config)) or 'none'}",
            flush=True,
        )
        print(
            "WebRTC public TURN URLs: "
            f"{', '.join(public_turn_urls_from_env()) or 'none'}",
            flush=True,
        )

    def _add_config_routes(self) -> None:
        for path in ("/", "/config", "/healthz"):
            route = self._app.router.add_get(path, self._handle_config)
            self._cors.add(route)

    def add_video_route(self, path: str, video_track: Any) -> None:
        def setup_video_peer(pc: RTCPeerConnection, _params: dict[str, Any]) -> None:
            pc.addTrack(self._media_relay.subscribe(video_track))

        self.add_offer_route(path, setup_video_peer)

    def add_input_route(self, path: str, on_message: MessageCallback) -> None:
        def setup_input_peer(pc: RTCPeerConnection, _params: dict[str, Any]) -> None:
            @pc.on("datachannel")
            def on_datachannel(channel):
                @channel.on("message")
                def on_datachannel_message(message):
                    result = on_message(message)
                    if inspect.isawaitable(result):
                        asyncio.create_task(result)

        self.add_offer_route(path, setup_input_peer)

    def add_offer_route(self, path: str, setup_peer: PeerSetupCallback) -> None:
        route = self._app.router.add_post(
            path,
            lambda request: self._handle_offer(request, setup_peer),
        )
        self._cors.add(route)

    def run(self, loop: asyncio.AbstractEventLoop | None = None) -> None:
        web.run_app(self._app, host=self.host, port=self.port, loop=loop)

    async def _handle_offer(
        self,
        request: web.Request,
        setup_peer: PeerSetupCallback,
    ) -> web.Response:
        try:
            params = await request.json()
            sdp_offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])
        except (json.JSONDecodeError, KeyError, TypeError) as exc:
            raise web.HTTPBadRequest(text=f"Invalid WebRTC offer: {exc}") from exc

        pc = RTCPeerConnection(configuration=self._ice_config)
        self._pcs.add(pc)
        request_path = request.path
        print(
            f"WebRTC offer received on {request_path} from {request.remote}; "
            f"remote candidates: {_candidate_summary(sdp_offer.sdp)}",
            flush=True,
        )

        @pc.on("icegatheringstatechange")
        def on_icegatheringstatechange():
            print(
                f"{request_path} ICE gathering state: {pc.iceGatheringState}",
                flush=True,
            )

        @pc.on("iceconnectionstatechange")
        def on_iceconnectionstatechange():
            print(
                f"{request_path} ICE connection state: {pc.iceConnectionState}",
                flush=True,
            )

        @pc.on("connectionstatechange")
        async def on_connectionstatechange():
            print(
                f"{request_path} connection state: {pc.connectionState}",
                flush=True,
            )
            if pc.connectionState in ("failed", "closed"):
                await pc.close()
                self._pcs.discard(pc)

        try:
            result = setup_peer(pc, params)
            if inspect.isawaitable(result):
                await result

            await pc.setRemoteDescription(sdp_offer)
            answer = await pc.createAnswer()
            await pc.setLocalDescription(answer)
            await self._wait_for_ice_gathering(pc)
            print(
                f"{request_path} local answer candidates: "
                f"{_candidate_summary(pc.localDescription.sdp)}",
                flush=True,
            )

            return web.Response(
                content_type="application/json",
                text=json.dumps(
                    {
                        "sdp": pc.localDescription.sdp,
                        "type": pc.localDescription.type,
                    }
                ),
            )
        except Exception:
            await pc.close()
            self._pcs.discard(pc)
            raise

    async def _handle_config(self, request: web.Request) -> web.Response:
        public_host = os.environ.get("WEBRTC_PUBLIC_HOST")
        if not public_host:
            public_host = request.host.rsplit(":", 1)[0]

        server_url = f"{request.scheme}://{public_host}:{self.port}"
        public_turn_urls = public_turn_urls_from_env()

        return web.json_response(
            {
                "status": "ok",
                "serverUrl": server_url,
                "offerUrl": f"{server_url}/offer",
                "inputOfferUrl": f"{server_url}/input_offer",
                "iceServers": [_ice_server_dict(public_turn_urls)]
                if public_turn_urls
                else [],
                "serverIceUrls": _ice_config_urls(self._ice_config),
            }
        )

    async def _wait_for_ice_gathering(self, pc: RTCPeerConnection) -> None:
        start = asyncio.get_event_loop().time()
        while pc.iceGatheringState != "complete":
            if asyncio.get_event_loop().time() - start > self._ice_gathering_timeout:
                break
            await asyncio.sleep(0.1)

    async def _on_shutdown(self, _app: web.Application) -> None:
        await asyncio.gather(*(pc.close() for pc in self._pcs), return_exceptions=True)
        self._pcs.clear()
