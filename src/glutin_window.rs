use gfx;
use gfx_device_gl;
use gfx_window_glutin;
use glutin;

use graphics::formats;

pub struct GlutinWindow {
    pub window: glutin::GlWindow,
    pub device: gfx_device_gl::Device,
    pub factory: gfx_device_gl::Factory,
    pub render_target_view: gfx::handle::RenderTargetView<
        gfx_device_gl::Resources,
        (gfx::format::R8_G8_B8_A8, gfx::format::Srgb),
    >,
    pub depth_stencil_view: gfx::handle::DepthStencilView<
        gfx_device_gl::Resources,
        (gfx::format::D24_S8, gfx::format::Unorm),
    >,
    pub events_loop: glutin::EventsLoop,
    pub encoder: gfx::Encoder<gfx_device_gl::Resources, gfx_device_gl::CommandBuffer>,
}

impl GlutinWindow {
    pub fn new(width: u32, height: u32) -> Self {
        let builder = glutin::WindowBuilder::new()
            .with_dimensions(width, height)
            .with_min_dimensions(width, height)
            .with_max_dimensions(width, height);
        let events_loop = glutin::EventsLoop::new();
        let context = glutin::ContextBuilder::new().with_vsync(true);
        let (window, device, mut factory, render_target_view, depth_stencil_view) =
            gfx_window_glutin::init::<formats::Colour, formats::Depth>(
                builder,
                context,
                &events_loop,
            );

        let encoder = factory.create_command_buffer().into();

        Self {
            window,
            device,
            factory,
            render_target_view,
            depth_stencil_view,
            events_loop,
            encoder,
        }
    }
}
