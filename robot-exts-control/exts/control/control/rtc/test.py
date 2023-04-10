import omni
import importlib
import carb.settings


class RTCTest():
    def __init__(self):
        manager = omni.kit.app.get_app().get_extension_manager()
        self._webrtc_was_enabled = manager.is_extension_enabled("omni.services.streamclient.webrtc")
        if not self._webrtc_was_enabled:
            manager.set_extension_enabled_immediate("omni.services.streamclient.webrtc", True)
        # self._webrtc_api = importlib.import_module("omni.physics.tensors")

    def test_main(self):
        from omni.services.client import AsyncClient
        from omni.services.streamclient.webrtc.services.browser_frontend import example_page, redirect_url, router_prefix

        frontend_port = carb.settings.get_settings().get_as_int("exts/omni.services.transport.server.http/port")
        frontend_prefix = f"http://localhost:{frontend_port}{router_prefix}"
        self._redirect_page_path = f"{frontend_prefix}{example_page}"
        self._client_page_path = f"{frontend_prefix}{redirect_url}"

        print("frontend_port", frontend_port)
        print("frontend_prefix", frontend_prefix)
        print("self._redirect_page_path", self._redirect_page_path)
        print("self._client_page_path", self._client_page_path)