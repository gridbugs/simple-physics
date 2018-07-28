pub mod formats {
    use gfx;
    pub type Colour = gfx::format::Srgba8;
    pub type Depth = gfx::format::DepthStencil;
}

mod consts {
    pub const QUAD_INDICES: [u16; 6] = [0, 1, 2, 2, 3, 0];
    pub const QUAD_COORDS: [[f32; 2]; 4] =
        [[0.0, 0.0], [0.0, 1.0], [1.0, 1.0], [1.0, 0.0]];
}

mod dimensions {
    use super::formats;
    use cgmath::{vec2, Vector2};
    use gfx;
    pub fn rtv_dimensions<R: gfx::Resources>(
        rtv: &gfx::handle::RenderTargetView<R, formats::Colour>,
    ) -> Vector2<u16> {
        let (width, height, _, _) = rtv.get_dimensions();
        vec2(width, height)
    }
}

mod buffer_types {
    use cgmath::Vector2;
    use gfx;
    gfx_vertex_struct!(QuadCorners {
        corner_zero_to_one: [f32; 2] = "a_CornerZeroToOne",
    });
    gfx_constant_struct!(WindowProperties {
        window_size_in_pixels: [f32; 2] = "u_WindowSizeInPixels",
    });
    pub fn update_window_properties<R, C>(
        properties_buffer: &gfx::handle::Buffer<R, WindowProperties>,
        window_size: Vector2<u16>,
        encoder: &mut gfx::Encoder<R, C>,
    ) where
        R: gfx::Resources,
        C: gfx::CommandBuffer<R>,
    {
        let properties = WindowProperties {
            window_size_in_pixels: [window_size.x as f32, window_size.y as f32],
        };
        encoder.update_constant_buffer(properties_buffer, &properties);
    }
}

mod buffer_alloc {
    use super::buffer_types;
    use super::consts;
    use gfx;
    pub fn create_instance_and_upload_buffers<R, F, T>(
        size: usize,
        factory: &mut F,
    ) -> Result<
        (gfx::handle::Buffer<R, T>, gfx::handle::Buffer<R, T>),
        gfx::buffer::CreationError,
    >
    where
        R: gfx::Resources,
        F: gfx::Factory<R> + gfx::traits::FactoryExt<R>,
    {
        let instance_buffer = factory.create_buffer(
            size,
            gfx::buffer::Role::Vertex,
            gfx::memory::Usage::Data,
            gfx::memory::Bind::TRANSFER_DST,
        )?;
        let upload_buffer = factory.create_upload_buffer(size)?;
        Ok((instance_buffer, upload_buffer))
    }
    pub fn create_corner_vertex_buffer_with_slice<R, F>(
        factory: &mut F,
    ) -> (
        gfx::handle::Buffer<R, buffer_types::QuadCorners>,
        gfx::Slice<R>,
    )
    where
        R: gfx::Resources,
        F: gfx::Factory<R> + gfx::traits::FactoryExt<R>,
    {
        let quad_corners_data = consts::QUAD_COORDS
            .iter()
            .map(|v| buffer_types::QuadCorners {
                corner_zero_to_one: *v,
            })
            .collect::<Vec<_>>();

        factory.create_vertex_buffer_with_slice(
            &quad_corners_data,
            &consts::QUAD_INDICES[..],
        )
    }
    pub fn create_window_property_buffer<R, F>(
        factory: &mut F,
    ) -> gfx::handle::Buffer<R, buffer_types::WindowProperties>
    where
        R: gfx::Resources,
        F: gfx::Factory<R> + gfx::traits::FactoryExt<R>,
    {
        factory.create_constant_buffer(1)
    }
}

mod instance_renderer {
    use super::buffer_alloc;
    use super::buffer_types;
    use super::formats;
    use super::InstanceWriter;
    use gfx;
    const MAX_NUM_INSTANCES: usize = 1024;
    pub struct ShaderBytes {
        pub vertex: &'static [u8],
        pub fragment: &'static [u8],
    }
    pub trait PipelineData<R: gfx::Resources>: gfx::pso::PipelineData<R> {
        type Instance: Copy + gfx::traits::Pod;
        type PipeInit: gfx::pso::PipelineInit<
            Meta = <Self as gfx::pso::PipelineData<R>>::Meta,
        >;
        fn new_data(
            corners: gfx::handle::Buffer<R, buffer_types::QuadCorners>,
            instances: gfx::handle::Buffer<R, Self::Instance>,
            properties: gfx::handle::Buffer<R, buffer_types::WindowProperties>,
            target: gfx::handle::RenderTargetView<R, formats::Colour>,
        ) -> Self;
        fn new_pipe() -> Self::PipeInit;
        fn instances(&self) -> &gfx::handle::Buffer<R, Self::Instance>;
        fn shader_bytes() -> ShaderBytes;
    }
    pub struct Renderer<R: gfx::Resources, D: PipelineData<R>> {
        bundle: gfx::Bundle<R, D>,
        num_quads: usize,
        instances_upload: gfx::handle::Buffer<R, D::Instance>,
    }

    impl<R: gfx::Resources, D: PipelineData<R>> Renderer<R, D> {
        pub fn new<F>(
            colour_rtv: &gfx::handle::RenderTargetView<R, formats::Colour>,
            window_properties: &gfx::handle::Buffer<R, buffer_types::WindowProperties>,
            factory: &mut F,
        ) -> Self
        where
            F: gfx::Factory<R> + gfx::traits::FactoryExt<R>,
        {
            let shader_bytes = <D as PipelineData<R>>::shader_bytes();
            let pso = factory
                .create_pipeline_simple(
                    shader_bytes.vertex,
                    shader_bytes.fragment,
                    <D as PipelineData<R>>::new_pipe(),
                )
                .expect("Failed to create pipeline");
            let (quad_corners_buf, slice) =
                buffer_alloc::create_corner_vertex_buffer_with_slice(factory);

            let (instances, instances_upload) =
                buffer_alloc::create_instance_and_upload_buffers(
                    MAX_NUM_INSTANCES,
                    factory,
                ).expect("Failed to create buffers");
            let data = <D as PipelineData<R>>::new_data(
                quad_corners_buf,
                instances,
                window_properties.clone(),
                colour_rtv.clone(),
            );
            let bundle = gfx::pso::bundle::Bundle::new(slice, pso, data);

            Self {
                bundle,
                num_quads: 0,
                instances_upload,
            }
        }

        pub fn instance_writer<F>(
            &mut self,
            factory: &mut F,
        ) -> InstanceWriter<R, D::Instance>
        where
            F: gfx::Factory<R> + gfx::traits::FactoryExt<R>,
        {
            let writer = factory
                .write_mapping(&self.instances_upload)
                .expect("Failed to map upload buffer");
            self.num_quads = 0;
            InstanceWriter {
                num_instances: &mut self.num_quads,
                bundle_slice_instances: &mut self.bundle.slice.instances,
                writer,
            }
        }

        pub fn encode<C>(&self, encoder: &mut gfx::Encoder<R, C>)
        where
            C: gfx::CommandBuffer<R>,
        {
            encoder
                .copy_buffer(
                    &self.instances_upload,
                    self.bundle.data.instances(),
                    0,
                    0,
                    self.num_quads,
                )
                .expect("Failed to copy instances");
            self.bundle.encode(encoder);
        }
    }
}

pub mod quad {
    use super::buffer_types;
    use super::formats;
    use super::instance_renderer::{self, PipelineData, ShaderBytes};
    use gfx;

    gfx_vertex_struct!(Instance {
        position_of_top_left_in_pixels: [f32; 2] = "i_PositionOfTopLeftInPixels",
        dimensions_in_pixels: [f32; 2] = "i_DimensionsInPixels",
        colour: [f32; 3] = "i_Colour",
    });

    gfx_pipeline!(pipe {
        quad_corners: gfx::VertexBuffer<buffer_types::QuadCorners> = (),
        instances: gfx::InstanceBuffer<Instance> = (),
        properties: gfx::ConstantBuffer<buffer_types::WindowProperties> = "WindowProperties",
        target: gfx::BlendTarget<formats::Colour> =
            ("Target", gfx::state::ColorMask::all(), gfx::preset::blend::ALPHA),
    });
    impl<R: gfx::Resources> PipelineData<R> for pipe::Data<R> {
        type Instance = Instance;
        type PipeInit = pipe::Init<'static>;
        fn new_data(
            corners: gfx::handle::Buffer<R, buffer_types::QuadCorners>,
            instances: gfx::handle::Buffer<R, Self::Instance>,
            properties: gfx::handle::Buffer<R, buffer_types::WindowProperties>,
            target: gfx::handle::RenderTargetView<R, formats::Colour>,
        ) -> Self {
            pipe::Data {
                quad_corners: corners,
                instances,
                properties,
                target,
            }
        }
        fn new_pipe() -> Self::PipeInit {
            pipe::new()
        }
        fn instances(&self) -> &gfx::handle::Buffer<R, Self::Instance> {
            &self.instances
        }
        fn shader_bytes() -> ShaderBytes {
            ShaderBytes {
                vertex: include_bytes!("shaders/quad/shader.150.vert"),
                fragment: include_bytes!("shaders/quad/shader.150.frag"),
            }
        }
    }

    pub type Renderer<R> = instance_renderer::Renderer<R, pipe::Data<R>>;
}

pub mod line_segment {
    use super::buffer_types;
    use super::formats;
    use super::instance_renderer::{self, PipelineData, ShaderBytes};
    use gfx;

    gfx_vertex_struct!(Instance {
        start: [f32; 2] = "i_Start",
        end: [f32; 2] = "i_End",
        colour: [f32; 3] = "i_Colour",
    });

    gfx_pipeline!(pipe {
        quad_corners: gfx::VertexBuffer<buffer_types::QuadCorners> = (),
        instances: gfx::InstanceBuffer<Instance> = (),
        properties: gfx::ConstantBuffer<buffer_types::WindowProperties> = "WindowProperties",
        target: gfx::BlendTarget<formats::Colour> =
            ("Target", gfx::state::ColorMask::all(), gfx::preset::blend::ALPHA),
    });

    impl<R: gfx::Resources> PipelineData<R> for pipe::Data<R> {
        type Instance = Instance;
        type PipeInit = pipe::Init<'static>;
        fn new_data(
            corners: gfx::handle::Buffer<R, buffer_types::QuadCorners>,
            instances: gfx::handle::Buffer<R, Self::Instance>,
            properties: gfx::handle::Buffer<R, buffer_types::WindowProperties>,
            target: gfx::handle::RenderTargetView<R, formats::Colour>,
        ) -> Self {
            pipe::Data {
                quad_corners: corners,
                instances,
                properties,
                target,
            }
        }
        fn new_pipe() -> Self::PipeInit {
            pipe::new()
        }
        fn instances(&self) -> &gfx::handle::Buffer<R, Self::Instance> {
            &self.instances
        }
        fn shader_bytes() -> ShaderBytes {
            ShaderBytes {
                vertex: include_bytes!("shaders/line_segment/shader.150.vert"),
                fragment: include_bytes!("shaders/line_segment/shader.150.frag"),
            }
        }
    }

    pub type Renderer<R> = instance_renderer::Renderer<R, pipe::Data<R>>;
}

use cgmath::Vector2;
use gfx;

pub struct InstanceWriter<'a, R: gfx::Resources, T: 'a + Copy> {
    num_instances: &'a mut usize,
    bundle_slice_instances: &'a mut Option<(u32, u32)>,
    writer: gfx::mapping::Writer<'a, R, T>,
}

pub struct InstanceWriterIterMut<'a, T: 'a> {
    num_instances: &'a mut usize,
    iter_mut: ::std::slice::IterMut<'a, T>,
}

impl<'a, T> Iterator for InstanceWriterIterMut<'a, T> {
    type Item = &'a mut T;
    fn next(&mut self) -> Option<Self::Item> {
        *self.num_instances += 1;
        self.iter_mut.next()
    }
}

impl<'a, R: gfx::Resources, T: Copy> Drop for InstanceWriter<'a, R, T> {
    fn drop(&mut self) {
        *self.bundle_slice_instances = Some((*self.num_instances as u32, 0));
    }
}

impl<'a, R: gfx::Resources, T: Copy> InstanceWriter<'a, R, T> {
    pub fn iter_mut(&mut self) -> InstanceWriterIterMut<T> {
        InstanceWriterIterMut {
            num_instances: &mut self.num_instances,
            iter_mut: self.writer.iter_mut(),
        }
    }
}

pub struct Frame<'a, R: gfx::Resources> {
    quad: InstanceWriter<'a, R, quad::Instance>,
    line_segment: InstanceWriter<'a, R, line_segment::Instance>,
}

impl<'a, R: gfx::Resources> Frame<'a, R> {
    pub fn updater(&mut self) -> FrameUpdater {
        FrameUpdater {
            quad: self.quad.iter_mut(),
            line_segment: self.line_segment.iter_mut(),
        }
    }
}

pub struct FrameUpdater<'a> {
    quad: InstanceWriterIterMut<'a, quad::Instance>,
    line_segment: InstanceWriterIterMut<'a, line_segment::Instance>,
}

impl<'a> FrameUpdater<'a> {
    pub fn axis_aligned_rect(
        &mut self,
        top_left: Vector2<f32>,
        size: Vector2<f32>,
        colour: [f32; 3],
    ) {
        if let Some(quad) = self.quad.next() {
            quad.position_of_top_left_in_pixels = top_left.into();
            quad.dimensions_in_pixels = size.into();
            quad.colour = colour;
        }
    }
    pub fn line_segment(
        &mut self,
        start: Vector2<f32>,
        end: Vector2<f32>,
        colour: [f32; 3],
    ) {
        if let Some(line_segment) = self.line_segment.next() {
            line_segment.start = start.into();
            line_segment.end = end.into();
            line_segment.colour = colour;
        }
    }
}

pub struct Renderer<R: gfx::Resources> {
    pub quad: quad::Renderer<R>,
    pub line_segment: line_segment::Renderer<R>,
}

impl<R: gfx::Resources> Renderer<R> {
    pub fn new<F, C>(
        colour_rtv: gfx::handle::RenderTargetView<R, formats::Colour>,
        factory: &mut F,
        encoder: &mut gfx::Encoder<R, C>,
    ) -> Self
    where
        F: gfx::Factory<R> + gfx::traits::FactoryExt<R>,
        C: gfx::CommandBuffer<R>,
    {
        let window_properties = buffer_alloc::create_window_property_buffer(factory);
        buffer_types::update_window_properties(
            &window_properties,
            dimensions::rtv_dimensions(&colour_rtv),
            encoder,
        );
        Self {
            quad: quad::Renderer::new(&colour_rtv, &window_properties, factory),
            line_segment: line_segment::Renderer::new(
                &colour_rtv,
                &window_properties,
                factory,
            ),
        }
    }
    pub fn prepare_frame<F>(&mut self, factory: &mut F) -> Frame<R>
    where
        F: gfx::Factory<R> + gfx::traits::FactoryExt<R>,
    {
        Frame {
            quad: self.quad.instance_writer(factory),
            line_segment: self.line_segment.instance_writer(factory),
        }
    }
    pub fn encode<C>(&self, encoder: &mut gfx::Encoder<R, C>)
    where
        C: gfx::CommandBuffer<R>,
    {
        self.quad.encode(encoder);
        self.line_segment.encode(encoder);
    }
}
