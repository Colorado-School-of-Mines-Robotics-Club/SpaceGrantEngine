import sys
import logging
from threading import Thread
from typing import Optional, Any

import glfw
import OpenGL.GL as gl
import imgui
from imgui.integrations.glfw import GlfwRenderer

from configuration import Config


class Engine(object):
    def __init__(self, config_file, headless=False, log_file: Optional[str] = None):
        """
        Highest level object for the SpaceGrantEngine library. Controls all submodules
        and provides interfaces. Handles concurrency and automatic parallelism.
        """
        super().__init__()
        # config instance to edit from imgui (if available)
        self._config: Config = Config(config_file=config_file)
        log_file = "space.log" if log_file is None else log_file
        self._logger: logging.Logger
        self._setup_logger(log_file)
        # flag for rendering
        self._headless: bool = headless

        self._window: Optional[Any] = None  # need to fill the type in
        self._impl: Optional[Any] = None  # need to fill the type in
        self._background_color = (0, 0, 0, 1)
        self._font_path = None  # Does not do anything currently
        self._render_thread: Optional[Thread] = None

        # do stuff if not headless
        if not self._headless:
            self._render_thread = Thread(name="render", target=self._render, args=())
            self._render_thread.start()

    # setup the logger
    def _setup_logger(self, log_file_name: str) -> None:
        logger = logging.getLogger()
        logger.setLevel(logging.INFO)
        formatter = logging.Formatter("%(asctime)s | %(levelname)s | %(message)s")
        # create stdout handler
        stdout_handler = logging.StreamHandler(sys.stdout)
        stdout_handler.setLevel(logging.DEBUG)
        stdout_handler.setFormatter(formatter)
        # create file handler
        file_handler = logging.FileHandler(log_file_name)
        file_handler.setLevel(logging.DEBUG)
        file_handler.setFormatter(formatter)
        # add handlers
        logger.addHandler(file_handler)
        logger.addHandler(stdout_handler)
        self._logger = logger

    def _impl_glfw_init(self, window_name="Space Grant Engine", width=1280, height=720):
        if not glfw.init():
            self._logger.error("Could not initialize OpenGL context")
            sys.exit(1)

        # OS X supports only forward-compatible core profiles from 3.2
        glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR, 3)
        glfw.window_hint(glfw.CONTEXT_VERSION_MINOR, 3)
        glfw.window_hint(glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE)

        glfw.window_hint(glfw.OPENGL_FORWARD_COMPAT, gl.GL_TRUE)

        # Create a windowed mode window and its OpenGL context
        window = glfw.create_window(int(width), int(height), window_name, None, None)
        glfw.make_context_current(window)

        if not window:
            glfw.terminate()
            self._logger.error("Could not initialize Window")
            sys.exit(1)

        return window

    def _render_frame(self):
        glfw.poll_events()
        self._impl.process_inputs()
        imgui.new_frame()

        gl.glClearColor(0.1, 0.1, 0.1, 1)
        gl.glClear(gl.GL_COLOR_BUFFER_BIT)

        if self._font is not None:
            imgui.push_font(self._font)
        self._frame_commands()
        if self._font is not None:
            imgui.pop_font()

        imgui.render()
        self._impl.render(imgui.get_draw_data())
        glfw.swap_buffers(self._window)

    def _render(self):
        imgui.create_context()
        self._window = self._impl_glfw_init()

        self._impl = GlfwRenderer(self._window)

        io = imgui.get_io()
        self._font = (
            io.fonts.add_font_from_file_ttf(self._font_path, 30)
            if self._font_path is not None
            else None
        )
        self._impl.refresh_font_texture()

        while not glfw.window_should_close(self._window):
            self._render_frame()

        self._impl.shutdown()
        glfw.terminate()

    def _frame_commands(self):
        io = imgui.get_io()

        if io.key_ctrl and io.keys_down[glfw.KEY_Q]:
            sys.exit(0)

        self._generate_menu_bar()

    def _generate_menu_bar(self):
        if imgui.begin_main_menu_bar():
            if imgui.begin_menu("File", True):
                clicked_quit, selected_quit = imgui.menu_item(
                    "Quit", "Ctrl+Q", False, True
                )

                if clicked_quit:
                    sys.exit(0)

                imgui.end_menu()

            if imgui.begin_menu("Config", True):
                # do something here

                imgui.end_menu()
            imgui.end_main_menu_bar()


if __name__ == "__main__":
    engine = Engine("test.json")
    