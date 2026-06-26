from image_listener.input_publisher import WebRtcInputPublisher


class _MessageSink:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, message) -> None:
        self.messages.append(message)


def _publisher_with_sink() -> tuple[WebRtcInputPublisher, _MessageSink]:
    publisher = object.__new__(WebRtcInputPublisher)
    sink = _MessageSink()
    publisher._hand_target_active_publisher = sink
    return publisher, sink


def test_explicit_deadman_release_is_published() -> None:
    publisher, sink = _publisher_with_sink()

    publisher._publish_hand_target_active(
        {
            "wristAvailable": True,
            "wristCommandEnabled": False,
        }
    )

    assert len(sink.messages) == 1
    assert sink.messages[0].data is False


def test_legacy_wrist_payload_defaults_to_active() -> None:
    publisher, sink = _publisher_with_sink()

    publisher._publish_hand_target_active({"wristAvailable": True})

    assert len(sink.messages) == 1
    assert sink.messages[0].data is True


def test_missing_wrist_does_not_change_deadman_state() -> None:
    publisher, sink = _publisher_with_sink()

    publisher._publish_hand_target_active({"wristAvailable": False})

    assert sink.messages == []
