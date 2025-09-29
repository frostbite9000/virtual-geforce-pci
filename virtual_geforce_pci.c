#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/timer.h>
#include <linux/dma-mapping.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/mm.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>

// Fix PCIBIOS constants
#ifndef PCIBIOS_SUCCESS
#define PCIBIOS_SUCCESS PCIBIOS_SUCCESSFUL
#endif

// GeForce constants from Bochs implementation
#define GEFORCE_PNPMMIO_SIZE        0x1000000
#define GEFORCE_CHANNEL_COUNT       32
#define GEFORCE_SUBCHANNEL_COUNT    8
#define GEFORCE_CACHE1_SIZE         64
#define VGA_CRTC_MAX                0x18
#define GEFORCE_CRTC_MAX            0x9F

// PCI Configuration
#define GEFORCE_VENDOR_ID           0x10DE  // NVIDIA
#define GEFORCE_DEVICE_ID_GF3       0x0202  // GeForce3 Ti 500
#define GEFORCE_DEVICE_ID_FX5900    0x0331  // GeForce FX 5900
#define GEFORCE_DEVICE_ID_6800      0x0045  // GeForce 6800 GT

// Memory sizes
#define GEFORCE_VRAM_SIZE_64MB      (64 * 1024 * 1024)
#define GEFORCE_VRAM_SIZE_128MB     (128 * 1024 * 1024)
#define GEFORCE_VRAM_SIZE_256MB     (256 * 1024 * 1024)

// 3D Pipeline constants (from Bochs)
#define MAX_VERTICES_PER_PRIMITIVE  3
#define MAX_TEXTURE_UNITS          16
#define VERTEX_CACHE_SIZE          16
#define TRANSFORM_PROGRAM_SIZE     544
#define TRANSFORM_CONSTANTS_SIZE   512
#define MAX_LIGHTS                 8

// Blending factors (from Bochs D3D implementation)
#define D3D_BLEND_ZERO             0x0000
#define D3D_BLEND_ONE              0x0001
#define D3D_BLEND_SRCCOLOR         0x0300
#define D3D_BLEND_INVSRCCOLOR      0x0301
#define D3D_BLEND_SRCALPHA         0x0302
#define D3D_BLEND_INVSRCALPHA      0x0303
#define D3D_BLEND_DESTALPHA        0x0304
#define D3D_BLEND_INVDESTALPHA     0x0305
#define D3D_BLEND_DESTCOLOR        0x0306
#define D3D_BLEND_INVDESTCOLOR     0x0307

// Depth functions
#define D3D_DEPTH_NEVER            0x0200
#define D3D_DEPTH_LESS             0x0201
#define D3D_DEPTH_EQUAL            0x0202
#define D3D_DEPTH_LEQUAL           0x0203
#define D3D_DEPTH_GREATER          0x0204
#define D3D_DEPTH_NOTEQUAL         0x0205
#define D3D_DEPTH_GEQUAL           0x0206
#define D3D_DEPTH_ALWAYS           0x0207

// GeForce card types
enum geforce_card_type {
    GEFORCE_3 = 0x20,
    GEFORCE_FX_5900 = 0x35,
    GEFORCE_6800 = 0x40
};

// CRTC structure from Bochs
struct geforce_crtc {
    u8 index;
    u8 reg[GEFORCE_CRTC_MAX + 1];
};

// Vertex structure for 3D pipeline
struct vertex_data {
    float position[4];      // x, y, z, w
    float normal[3];        // nx, ny, nz
    float texcoord[4][4];   // up to 4 texture coordinate sets
    float color[4];         // r, g, b, a
};

// Light structure
struct light_data {
    float diffuse_color[3];
    float infinite_direction[3];
    bool enabled;
};

// Texture state
struct texture_state {
    u32 offset;
    u32 format;
    u32 address_mode;
    u32 control0, control1, control3;
    u32 filter;
    u32 image_rect;
    u32 palette;
    u32 width, height;
    u32 pitch;
};

// 3D Channel state (enhanced from Bochs gf_channel)
struct geforce_channel {
    u32 subr_return;
    bool subr_active;
    struct {
        u32 mthd;
        u32 subc;
        u32 mcnt;
        bool ni;
    } dma_state;
    
    struct {
        u32 object;
        u8 engine;
        u32 notifier;
    } subchannel[GEFORCE_SUBCHANNEL_COUNT];
    
    bool notify_pending;
    u32 notify_type;
    
    // 2D Surface state
    u32 s2d_img_src;
    u32 s2d_img_dst;
    u32 s2d_color_fmt;
    u32 s2d_color_bytes;
    u32 s2d_pitch;
    u32 s2d_ofs_src;
    u32 s2d_ofs_dst;
    
    // 3D Rendering state (from Bochs D3D implementation)
    u32 d3d_a_obj;
    u32 d3d_b_obj;
    u32 d3d_color_obj;
    u32 d3d_zeta_obj;
    u32 d3d_vertex_a_obj;
    u32 d3d_vertex_b_obj;
    u32 d3d_report_obj;
    
    // Surface configuration
    u32 d3d_clip_horizontal;
    u32 d3d_clip_vertical;
    u32 d3d_surface_format;
    u32 d3d_color_bytes;
    u32 d3d_depth_bytes;
    u32 d3d_surface_pitch_a;
    u32 d3d_surface_pitch_z;
    u32 d3d_window_offset;
    u32 d3d_surface_color_offset;
    u32 d3d_surface_zeta_offset;
    
    // Blending state
    u32 d3d_blend_enable;
    u32 d3d_blend_func_sfactor;
    u32 d3d_blend_func_dfactor;
    
    // Depth and stencil state
    u32 d3d_depth_test_enable;
    u32 d3d_depth_write_enable;
    u32 d3d_depth_func;
    float d3d_clip_min;
    float d3d_clip_max;
    
    // Culling state
    u32 d3d_cull_face_enable;
    u32 d3d_cull_face;
    u32 d3d_front_face;
    
    // Lighting state
    u32 d3d_lighting_enable;
    u32 d3d_light_enable_mask;
    u32 d3d_shade_mode;
    float d3d_scene_ambient_color[4];
    struct light_data d3d_lights[MAX_LIGHTS];
    
    // Transformation matrices
    float d3d_inverse_model_view_matrix[12];
    float d3d_composite_matrix[16];
    
    // Vertex shader state
    u32 d3d_shader_program;
    u32 d3d_shader_obj;
    u32 d3d_shader_offset;
    u32 d3d_transform_program[TRANSFORM_PROGRAM_SIZE][4];
    float d3d_transform_constant[TRANSFORM_CONSTANTS_SIZE][4];
    u32 d3d_transform_execution_mode;
    u32 d3d_transform_program_load;
    u32 d3d_transform_program_start;
    u32 d3d_transform_constant_load;
    
    // Viewport state
    u32 d3d_viewport_horizontal;
    u32 d3d_viewport_vertical;
    float d3d_viewport_offset[4];
    float d3d_viewport_scale[4];
    
    // Vertex data arrays
    u32 d3d_vertex_data_array_offset[16];
    u32 d3d_vertex_data_array_format_type[16];
    u32 d3d_vertex_data_array_format_size[16];
    u32 d3d_vertex_data_array_format_stride[16];
    bool d3d_vertex_data_array_format_dx[16];
    
    // Primitive assembly state
    u32 d3d_begin_end;
    bool d3d_primitive_done;
    bool d3d_triangle_flip;
    u32 d3d_vertex_index;
    u32 d3d_attrib_index;
    u32 d3d_comp_index;
    
    // Vertex cache
    struct vertex_data d3d_vertex_cache[VERTEX_CACHE_SIZE];
    float d3d_vertex_data[4][16][4];
    
    // Index array
    u32 d3d_index_array_offset;
    u32 d3d_index_array_dma;
    
    // Texture state
    struct texture_state d3d_textures[MAX_TEXTURE_UNITS];
    
    // Clear values
    u32 d3d_zstencil_clear_value;
    u32 d3d_color_clear_value;
    u32 d3d_clear_surface;
    
    // Vertex attributes
    u32 d3d_attrib_color;
    u32 d3d_attrib_tex_coord[10];
    float d3d_normal[3];
    float d3d_diffuse_color[4];
    float d3d_texcoord[4][4];
    
    // Synchronization
    u32 d3d_semaphore_obj;
    u32 d3d_semaphore_offset;
    
    // ROP state
    u8 rop;
    u32 beta;
    
    // Clipping
    u32 clip_yx;
    u32 clip_hw;
    
    // Color key
    u32 chroma_color_fmt;
    u32 chroma_color;
    
    // Pattern state
    u32 patt_shape;
    u32 patt_type;
    u32 patt_bg_color;
    u32 patt_fg_color;
    bool patt_data_mono[64];
    u32 patt_data_color[64];
};

// Main GeForce device structure
struct virtual_geforce_dev {
    struct pci_dev *pci_dev;
    struct pci_bus *virtual_bus;
    struct device *parent_dev;
    
    // Memory regions
    void __iomem *mmio_base;
    void *vram_base;
    dma_addr_t vram_dma_addr;
    
    // Device configuration
    enum geforce_card_type card_type;
    u32 vram_size;
    u32 bar2_size;
    u32 memsize_mask;
    u32 ramin_flip;
    u32 class_mask;
    
    // GeForce registers (from Bochs implementation)
    struct geforce_crtc crtc;
    u32 mc_intr_en;
    u32 mc_enable;
    u32 bus_intr;
    u32 bus_intr_en;
    u32 fifo_intr;
    u32 fifo_intr_en;
    u32 fifo_ramht;
    u32 fifo_ramfc;
    u32 fifo_ramro;
    u32 fifo_mode;
    u32 fifo_cache1_push1;
    u32 fifo_cache1_put;
    u32 fifo_cache1_dma_push;
    u32 fifo_cache1_dma_instance;
    u32 fifo_cache1_dma_put;
    u32 fifo_cache1_dma_get;
    u32 fifo_cache1_ref_cnt;
    u32 fifo_cache1_pull0;
    u32 fifo_cache1_semaphore;
    u32 fifo_cache1_get;
    u32 fifo_grctx_instance;
    u32 fifo_cache1_method[GEFORCE_CACHE1_SIZE];
    u32 fifo_cache1_data[GEFORCE_CACHE1_SIZE];
    
    // Graphics engine registers
    u32 graph_intr;
    u32 graph_nsource;
    u32 graph_intr_en;
    u32 graph_ctx_switch1;
    u32 graph_ctx_switch2;
    u32 graph_ctx_switch4;
    u32 graph_ctxctl_cur;
    u32 graph_status;
    u32 graph_trapped_addr;
    u32 graph_trapped_data;
    u32 graph_notify;
    u32 graph_fifo;
    u32 graph_channel_ctx_table;
    u32 graph_offset0;
    u32 graph_pitch0;
    
    // CRTC registers
    u32 crtc_intr;
    u32 crtc_intr_en;
    u32 crtc_start;
    u32 crtc_config;
    u32 crtc_cursor_offset;
    u32 crtc_cursor_config;
    
    // RAMDAC registers
    u32 ramdac_cu_start_pos;
    u32 ramdac_vpll;
    u32 ramdac_vpll_b;
    u32 ramdac_pll_select;
    u32 ramdac_general_control;
    
    // Timer registers
    u32 timer_intr;
    u32 timer_intr_en;
    u32 timer_num;
    u32 timer_den;
    u64 timer_inittime1;
    u64 timer_inittime2;
    u32 timer_alarm;
    
    // Straps
    u32 straps0_primary;
    u32 straps0_primary_original;
    
    // Display state
    u32 svga_xres;
    u32 svga_yres;
    u32 svga_pitch;
    u32 svga_bpp;
    bool svga_needs_update_mode;
    
    // Channels with enhanced 3D support
    struct geforce_channel channels[GEFORCE_CHANNEL_COUNT];
    
    // Work queue for processing
    struct workqueue_struct *workqueue;
    struct work_struct fifo_work;
    struct work_struct d3d_work;
    
    // IRQ handling
    int irq_line;
    spinlock_t reg_lock;
    
    // 3D Pipeline state
    struct mutex d3d_mutex;
    bool d3d_pipeline_active;
    u32 active_3d_channel;
};

static struct virtual_geforce_dev *global_geforce_dev = NULL;

// Math helper functions (from Bochs) - Fixed to avoid SSE issues
static u32 __attribute__((__noinline__)) float_to_uint32_bits(float val)
{
    volatile union { u32 i; float f; } u;
    u.f = val;
    return u.i;
}

// Use pointer to return float to avoid SSE register return
static void __attribute__((__noinline__)) uint32_bits_to_float_ptr(u32 val, float *result)
{
    volatile union { u32 i; float f; } u;
    u.i = val;
    *result = u.f;
}

// Color format conversion (from Bochs)
static inline u32 color_565_to_888(u16 value)
{
    u32 r = (value >> 11) & 0x1F;
    u32 g = (value >> 5) & 0x3F;
    u32 b = value & 0x1F;
    
    r = (r << 3) | (r >> 2);
    g = (g << 2) | (g >> 4);
    b = (b << 3) | (b >> 2);
    
    return (r << 16) | (g << 8) | b;
}

static inline u16 color_888_to_565(u32 value)
{
    return (((value >> 19) & 0x1F) << 11) | 
           (((value >> 10) & 0x3F) << 5) | 
           ((value >> 3) & 0x1F);
}

// Register access helpers (adapted from Bochs)
static inline u32 geforce_reg_read32(struct virtual_geforce_dev *gdev, u32 offset)
{
    if (gdev->mmio_base)
        return readl(gdev->mmio_base + offset);
    return 0;
}

static inline void geforce_reg_write32(struct virtual_geforce_dev *gdev, u32 offset, u32 value)
{
    if (gdev->mmio_base)
        writel(value, gdev->mmio_base + offset);
}

// VRAM access helpers
static inline u8 geforce_vram_read8(struct virtual_geforce_dev *gdev, u32 offset)
{
    if (gdev->vram_base && offset < gdev->vram_size)
        return *((u8*)gdev->vram_base + offset);
    return 0;
}

static inline void geforce_vram_write8(struct virtual_geforce_dev *gdev, u32 offset, u8 value)
{
    if (gdev->vram_base && offset < gdev->vram_size)
        *((u8*)gdev->vram_base + offset) = value;
}

static inline u32 geforce_vram_read32(struct virtual_geforce_dev *gdev, u32 offset)
{
    if (gdev->vram_base && offset + 3 < gdev->vram_size)
        return *((u32*)((u8*)gdev->vram_base + offset));
    return 0;
}

static inline void geforce_vram_write32(struct virtual_geforce_dev *gdev, u32 offset, u32 value)
{
    if (gdev->vram_base && offset + 3 < gdev->vram_size)
        *((u32*)((u8*)gdev->vram_base + offset)) = value;
}

// RAMIN access (adapted from Bochs)
static inline u32 geforce_ramin_read32(struct virtual_geforce_dev *gdev, u32 offset)
{
    return geforce_vram_read32(gdev, offset ^ gdev->ramin_flip);
}

static inline void geforce_ramin_write32(struct virtual_geforce_dev *gdev, u32 offset, u32 value)
{
    geforce_vram_write32(gdev, offset ^ gdev->ramin_flip, value);
}

// DMA object lookup (from Bochs)
static u32 geforce_dma_lookup(struct virtual_geforce_dev *gdev, u32 object, u32 address)
{
    // Simplified DMA translation - in real hardware this would be more complex
    return address & gdev->memsize_mask;
}

// Pixel operations (adapted from Bochs)
static u32 geforce_get_pixel(struct virtual_geforce_dev *gdev, u32 obj, u32 offset, u32 x, u32 color_bytes)
{
    u32 addr = geforce_dma_lookup(gdev, obj, offset + x * color_bytes);
    
    switch (color_bytes) {
        case 1:
            return geforce_vram_read8(gdev, addr);
        case 2:
            return geforce_vram_read32(gdev, addr & ~3) >> ((addr & 3) * 8);
        case 4:
            return geforce_vram_read32(gdev, addr);
        default:
            return 0;
    }
}

static void geforce_put_pixel(struct virtual_geforce_dev *gdev, struct geforce_channel *ch, 
                             u32 offset, u32 x, u32 value)
{
    u32 addr = geforce_dma_lookup(gdev, ch->s2d_img_dst, offset + x * ch->s2d_color_bytes);
    
    switch (ch->s2d_color_bytes) {
        case 1:
            geforce_vram_write8(gdev, addr, value);
            break;
        case 2:
            {
                u32 shift = (addr & 3) * 8;
                u32 mask = ~(0xFFFF << shift);
                u32 old_val = geforce_vram_read32(gdev, addr & ~3);
                geforce_vram_write32(gdev, addr & ~3, (old_val & mask) | ((value & 0xFFFF) << shift));
            }
            break;
        case 4:
            geforce_vram_write32(gdev, addr, value);
            break;
    }
}

// 3D Pipeline: Vertex Transformation (adapted from Bochs d3d_transform)
static void __attribute__((__noinline__)) geforce_d3d_transform_vertex(struct geforce_channel *ch, float input[4], float output[4])
{
    int i, j;
    
    // Apply composite transformation matrix
    for (i = 0; i < 4; i++) {
        volatile float temp = 0.0f;
        for (j = 0; j < 4; j++) {
            volatile float matrix_val = ch->d3d_composite_matrix[i * 4 + j];
            volatile float input_val = input[j];
            temp += matrix_val * input_val;
        }
        output[i] = temp;
    }
    
    // Debug print disabled to avoid SSE register issues
    // printk(KERN_DEBUG "GeForce: Transformed vertex (%.2f,%.2f,%.2f,%.2f) -> (%.2f,%.2f,%.2f,%.2f)\n",
    //        input[0], input[1], input[2], input[3],
    //        output[0], output[1], output[2], output[3]);
}

// 3D Pipeline: Vertex Shader (adapted from Bochs d3d_vertex_shader)
static void geforce_d3d_vertex_shader(struct virtual_geforce_dev *gdev, struct geforce_channel *ch, 
                                      float input[16][4], float output[16][4])
{
    int i, j;
    
    // Simple pass-through vertex shader for now
    // In a full implementation, this would execute the vertex program
    for (i = 0; i < 16; i++) {
        for (j = 0; j < 4; j++) {
            output[i][j] = input[i][j];
        }
    }
    
    // Apply transformation to position (attribute 0)
    if (ch->d3d_transform_execution_mode) {
        geforce_d3d_transform_vertex(ch, input[0], output[0]);
    }
    
    // Apply lighting if enabled
    if (ch->d3d_lighting_enable) {
        float normal[3] = { input[2][0], input[2][1], input[2][2] };  // Normal from attribute 2
        float final_color[4] = { ch->d3d_scene_ambient_color[0], 
                                ch->d3d_scene_ambient_color[1], 
                                ch->d3d_scene_ambient_color[2], 
                                ch->d3d_scene_ambient_color[3] };
        
        // Process each enabled light
        for (i = 0; i < MAX_LIGHTS; i++) {
            if (ch->d3d_light_enable_mask & (1 << i)) {
                float dot = normal[0] * ch->d3d_lights[i].infinite_direction[0] +
                           normal[1] * ch->d3d_lights[i].infinite_direction[1] +
                           normal[2] * ch->d3d_lights[i].infinite_direction[2];
                
                if (dot > 0.0f) {
                    final_color[0] += dot * ch->d3d_lights[i].diffuse_color[0];
                    final_color[1] += dot * ch->d3d_lights[i].diffuse_color[1];
                    final_color[2] += dot * ch->d3d_lights[i].diffuse_color[2];
                }
            }
        }
        
        // Clamp colors
        for (j = 0; j < 3; j++) {
            if (final_color[j] > 1.0f) final_color[j] = 1.0f;
            if (final_color[j] < 0.0f) final_color[j] = 0.0f;
        }
        
        // Store final color in output attribute 3
        output[3][0] = final_color[0];
        output[3][1] = final_color[1];
        output[3][2] = final_color[2];
        output[3][3] = final_color[3];
    }
    
    printk(KERN_DEBUG "GeForce: Vertex shader processed\n");
}

// 3D Pipeline: Texture Sampling (adapted from Bochs d3d_sample_texture)
static void geforce_d3d_sample_texture(struct virtual_geforce_dev *gdev, struct geforce_channel *ch,
                                       u32 tex_unit, float str[3], float color[4])
{
    struct texture_state *tex = &ch->d3d_textures[tex_unit];
    
    if (tex_unit >= MAX_TEXTURE_UNITS || tex->offset == 0) {
        color[0] = color[1] = color[2] = color[3] = 1.0f;
        return;
    }
    
    // Simple nearest neighbor sampling
    int x = (int)(str[0] * tex->width) % tex->width;
    int y = (int)(str[1] * tex->height) % tex->height;
    
    if (x < 0) x = 0;
    if (y < 0) y = 0;
    
    u32 texel_offset = geforce_dma_lookup(gdev, tex->offset, y * tex->pitch + x * 4);
    u32 texel = geforce_vram_read32(gdev, texel_offset);
    
    // Convert to float colors
    color[0] = ((texel >> 16) & 0xFF) / 255.0f;  // R
    color[1] = ((texel >> 8) & 0xFF) / 255.0f;   // G
    color[2] = (texel & 0xFF) / 255.0f;          // B
    color[3] = ((texel >> 24) & 0xFF) / 255.0f;  // A
    
    printk(KERN_DEBUG "GeForce: Sampled texture %d at (%.2f,%.2f) -> RGBA(%.2f,%.2f,%.2f,%.2f)\n",
           tex_unit, str[0], str[1], color[0], color[1], color[2], color[3]);
}

// 3D Pipeline: Pixel Shader (adapted from Bochs d3d_pixel_shader)
static void geforce_d3d_pixel_shader(struct virtual_geforce_dev *gdev, struct geforce_channel *ch,
                                     float input[16][4], float output[4])
{
    int i;
    
    // Start with vertex color (attribute 3)
    output[0] = input[3][0];
    output[1] = input[3][1];
    output[2] = input[3][2];
    output[3] = input[3][3];
    
    // Sample and modulate textures
    for (i = 0; i < MAX_TEXTURE_UNITS; i++) {
        if (ch->d3d_textures[i].offset != 0) {
            float tex_color[4];
            float tex_coords[3] = { input[8 + i][0], input[8 + i][1], input[8 + i][2] };
            
            geforce_d3d_sample_texture(gdev, ch, i, tex_coords, tex_color);
            
            // Simple modulation
            output[0] *= tex_color[0];
            output[1] *= tex_color[1];
            output[2] *= tex_color[2];
            output[3] *= tex_color[3];
        }
    }
    
    // Clamp final color
    for (i = 0; i < 4; i++) {
        if (output[i] > 1.0f) output[i] = 1.0f;
        if (output[i] < 0.0f) output[i] = 0.0f;
    }
    
    printk(KERN_DEBUG "GeForce: Pixel shader output RGBA(%.2f,%.2f,%.2f,%.2f)\n",
           output[0], output[1], output[2], output[3]);
}

// 3D Pipeline: Blending (adapted from Bochs d3d_blend) - simplified to avoid SSE
static void geforce_d3d_blend_factor_ptr(struct geforce_channel *ch, u32 factor,
                                      float src_val, float src_alpha, float dst_val, float dst_alpha, float *result)
{
    // Simplified implementation to avoid SSE register issues
    // For now, just implement the most common cases to get compilation working
    switch (factor) {
        case D3D_BLEND_ZERO:
            *result = 0.0f;
            return;
        case D3D_BLEND_ONE:
            *result = 1.0f;
            return;
        case D3D_BLEND_SRCCOLOR:
        case D3D_BLEND_SRCALPHA:
            *result = src_val;  // Use src_val for both cases for simplicity
            return;
        case D3D_BLEND_DESTCOLOR:
        case D3D_BLEND_DESTALPHA:
            *result = dst_val;  // Use dst_val for both cases for simplicity
            return;
        default:
            // For inverse cases and others, just use a fixed value to avoid float math
            *result = 1.0f;
            return;
    }
}



// 3D Pipeline: Depth Test
static bool geforce_d3d_depth_test(struct geforce_channel *ch, float z, float existing_z)
{
    if (!ch->d3d_depth_test_enable)
        return true;
    
    switch (ch->d3d_depth_func) {
        case D3D_DEPTH_NEVER:
            return false;
        case D3D_DEPTH_LESS:
            return z < existing_z;
        case D3D_DEPTH_EQUAL:
            return z == existing_z;
        case D3D_DEPTH_LEQUAL:
            return z <= existing_z;
        case D3D_DEPTH_GREATER:
            return z > existing_z;
        case D3D_DEPTH_NOTEQUAL:
            return z != existing_z;
        case D3D_DEPTH_GEQUAL:
            return z >= existing_z;
        case D3D_DEPTH_ALWAYS:
            return true;
        default:
            return true;
    }
}

// 3D Pipeline: Triangle Rasterization (adapted from Bochs d3d_triangle) - Fixed SSE issue
static void __attribute__((__noinline__)) geforce_d3d_rasterize_triangle(struct virtual_geforce_dev *gdev, struct geforce_channel *ch,
                                           struct vertex_data *v0, struct vertex_data *v1, struct vertex_data *v2)
{
    int x, y;
    float min_x, max_x, min_y, max_y;
    
    // Calculate bounding box
    min_x = (v0->position[0] < v1->position[0]) ? v0->position[0] : v1->position[0];
    min_x = (min_x < v2->position[0]) ? min_x : v2->position[0];
    
    max_x = (v0->position[0] > v1->position[0]) ? v0->position[0] : v1->position[0];
    max_x = (max_x > v2->position[0]) ? max_x : v2->position[0];
    
    min_y = (v0->position[1] < v1->position[1]) ? v0->position[1] : v1->position[1];
    min_y = (min_y < v2->position[1]) ? min_y : v2->position[1];
    
    max_y = (v0->position[1] > v1->position[1]) ? v0->position[1] : v1->position[1];
    max_y = (max_y > v2->position[1]) ? max_y : v2->position[1];
    
    // Clip to viewport
    if (min_x < 0) min_x = 0;
    if (min_y < 0) min_y = 0;
    if (max_x >= ch->d3d_viewport_horizontal) max_x = ch->d3d_viewport_horizontal - 1;
    if (max_y >= ch->d3d_viewport_vertical) max_y = ch->d3d_viewport_vertical - 1;
    
    // Debug print disabled to avoid SSE register issues
    // printk(KERN_DEBUG "GeForce: Rasterizing triangle bounds (%.1f,%.1f)-(%.1f,%.1f)\n",
    //        min_x, min_y, max_x, max_y);
    
    // Rasterize triangle using barycentric coordinates
    for (y = (int)min_y; y <= (int)max_y; y++) {
        for (x = (int)min_x; x <= (int)max_x; x++) {
            // Compute barycentric coordinates
            float p[2] = { (float)x, (float)y };
            float v0v1[2] = { v1->position[0] - v0->position[0], v1->position[1] - v0->position[1] };
            float v0v2[2] = { v2->position[0] - v0->position[0], v2->position[1] - v0->position[1] };
            float v0p[2] = { p[0] - v0->position[0], p[1] - v0->position[1] };
            
            float dot00 = v0v2[0] * v0v2[0] + v0v2[1] * v0v2[1];
            float dot01 = v0v2[0] * v0v1[0] + v0v2[1] * v0v1[1];
            float dot02 = v0v2[0] * v0p[0] + v0v2[1] * v0p[1];
            float dot11 = v0v1[0] * v0v1[0] + v0v1[1] * v0v1[1];
            float dot12 = v0v1[0] * v0p[0] + v0v1[1] * v0p[1];
            
            float inv_denom = 1.0f / (dot00 * dot11 - dot01 * dot01);
            float u = (dot11 * dot02 - dot01 * dot12) * inv_denom;
            float v = (dot00 * dot12 - dot01 * dot02) * inv_denom;
            
            // Check if point is inside triangle
            if (u >= 0 && v >= 0 && u + v <= 1) {
                float w = 1.0f - u - v;
                
                // Interpolate vertex attributes
                float interpolated_attrs[16][4];
                int attr, comp;
                
                for (attr = 0; attr < 16; attr++) {
                    for (comp = 0; comp < 4; comp++) {
                        interpolated_attrs[attr][comp] = 
                            w * v0->color[comp] + u * v1->color[comp] + v * v2->color[comp];
                    }
                }
                
                // Interpolate depth
                float z = w * v0->position[2] + u * v1->position[2] + v * v2->position[2];
                
                // Depth test
                u32 depth_addr = geforce_dma_lookup(gdev, ch->d3d_zeta_obj, 
                                                   ch->d3d_surface_zeta_offset + y * ch->d3d_surface_pitch_z + x * ch->d3d_depth_bytes);
                float existing_depth = 0.0f;
                
                if (ch->d3d_depth_bytes == 4) {
                    u32 depth_val = geforce_vram_read32(gdev, depth_addr);
                    uint32_bits_to_float_ptr(depth_val, &existing_depth);
                }
                
                if (geforce_d3d_depth_test(ch, z, existing_depth)) {
                    // Write depth if enabled
                    if (ch->d3d_depth_write_enable && ch->d3d_depth_bytes == 4) {
                        geforce_vram_write32(gdev, depth_addr, float_to_uint32_bits(z));
                    }
                    
                    // Run pixel shader
                    float pixel_color[4];
                    geforce_d3d_pixel_shader(gdev, ch, interpolated_attrs, pixel_color);
                    
                    // Blending
                    if (ch->d3d_blend_enable) {
                        u32 color_addr = geforce_dma_lookup(gdev, ch->d3d_color_obj,
                                                           ch->d3d_surface_color_offset + y * ch->d3d_surface_pitch_a + x * ch->d3d_color_bytes);
                        
                        if (ch->d3d_color_bytes == 4) {
                            u32 existing_color = geforce_vram_read32(gdev, color_addr);
                            float dst_color[4] = {
                                ((existing_color >> 16) & 0xFF) / 255.0f,
                                ((existing_color >> 8) & 0xFF) / 255.0f,
                                (existing_color & 0xFF) / 255.0f,
                                ((existing_color >> 24) & 0xFF) / 255.0f
                            };
                            
                            // Apply blending
                            float src_factor_r, src_factor_g, src_factor_b;
                            float dst_factor_r, dst_factor_g, dst_factor_b;
                            
                            geforce_d3d_blend_factor_ptr(ch, ch->d3d_blend_func_sfactor, pixel_color[0], pixel_color[3], dst_color[0], dst_color[3], &src_factor_r);
                            geforce_d3d_blend_factor_ptr(ch, ch->d3d_blend_func_sfactor, pixel_color[1], pixel_color[3], dst_color[1], dst_color[3], &src_factor_g);
                            geforce_d3d_blend_factor_ptr(ch, ch->d3d_blend_func_sfactor, pixel_color[2], pixel_color[3], dst_color[2], dst_color[3], &src_factor_b);
                            
                            geforce_d3d_blend_factor_ptr(ch, ch->d3d_blend_func_dfactor, pixel_color[0], pixel_color[3], dst_color[0], dst_color[3], &dst_factor_r);
                            geforce_d3d_blend_factor_ptr(ch, ch->d3d_blend_func_dfactor, pixel_color[1], pixel_color[3], dst_color[1], dst_color[3], &dst_factor_g);
                            geforce_d3d_blend_factor_ptr(ch, ch->d3d_blend_func_dfactor, pixel_color[2], pixel_color[3], dst_color[2], dst_color[3], &dst_factor_b);
                            
                            pixel_color[0] = pixel_color[0] * src_factor_r + dst_color[0] * dst_factor_r;
                            pixel_color[1] = pixel_color[1] * src_factor_g + dst_color[1] * dst_factor_g;
                            pixel_color[2] = pixel_color[2] * src_factor_b + dst_color[2] * dst_factor_b;
                        }
                    }
                    
                    // Write final color
                    u32 color_addr = geforce_dma_lookup(gdev, ch->d3d_color_obj,
                                                       ch->d3d_surface_color_offset + y * ch->d3d_surface_pitch_a + x * ch->d3d_color_bytes);
                    
                    if (ch->d3d_color_bytes == 4) {
                        u32 final_color = ((u32)(pixel_color[3] * 255) << 24) |
                                         ((u32)(pixel_color[0] * 255) << 16) |
                                         ((u32)(pixel_color[1] * 255) << 8) |
                                         ((u32)(pixel_color[2] * 255));
                        geforce_vram_write32(gdev, color_addr, final_color);
                    }
                }
            }
        }
    }
    
    printk(KERN_DEBUG "GeForce: Triangle rasterization complete\n");
}

// 3D Pipeline: Surface Clear (adapted from Bochs d3d_clear_surface)
static void geforce_d3d_clear_surface(struct virtual_geforce_dev *gdev, struct geforce_channel *ch)
{
    u32 x, y;
    u32 width = ch->d3d_viewport_horizontal;
    u32 height = ch->d3d_viewport_vertical;
    
    printk(KERN_INFO "GeForce: Clearing surface %dx%d\n", width, height);
    
    // Clear color buffer
    if (ch->d3d_color_obj && (ch->d3d_clear_surface & 0x01)) {
        for (y = 0; y < height; y++) {
            for (x = 0; x < width; x++) {
                u32 addr = geforce_dma_lookup(gdev, ch->d3d_color_obj,
                                             ch->d3d_surface_color_offset + y * ch->d3d_surface_pitch_a + x * ch->d3d_color_bytes);
                if (ch->d3d_color_bytes == 4) {
                    geforce_vram_write32(gdev, addr, ch->d3d_color_clear_value);
                }
            }
        }
    }
    
    // Clear depth/stencil buffer
    if (ch->d3d_zeta_obj && (ch->d3d_clear_surface & 0x02)) {
        for (y = 0; y < height; y++) {
            for (x = 0; x < width; x++) {
                u32 addr = geforce_dma_lookup(gdev, ch->d3d_zeta_obj,
                                             ch->d3d_surface_zeta_offset + y * ch->d3d_surface_pitch_z + x * ch->d3d_depth_bytes);
                if (ch->d3d_depth_bytes == 4) {
                    geforce_vram_write32(gdev, addr, ch->d3d_zstencil_clear_value);
                }
            }
        }
    }
}

// 3D Command Execution (adapted from Bochs execute_d3d)
static void geforce_execute_d3d_command(struct virtual_geforce_dev *gdev, struct geforce_channel *ch,
                                        u32 cls, u32 method, u32 param)
{
    union {
        u32 param_integer;
        float param_float;
    } u;
    u.param_integer = param;
    
    printk(KERN_DEBUG "GeForce: D3D command class=0x%04x method=0x%04x param=0x%08x\n", 
           cls, method, param);
    
    switch (method) {
        // Surface configuration
        case 0x0300:  // Set color surface
            ch->d3d_color_obj = param;
            break;
        case 0x0304:  // Set depth surface
            ch->d3d_zeta_obj = param;
            break;
        case 0x0308:  // Set surface format
            ch->d3d_surface_format = param;
            ch->d3d_color_bytes = (param & 0x0F) == 0x05 ? 4 : 2;  // A8R8G8B8 vs R5G6B5
            ch->d3d_depth_bytes = ((param >> 4) & 0x0F) == 0x03 ? 4 : 2;  // Z24S8 vs Z16
            break;
        case 0x030C:  // Set surface pitch
            ch->d3d_surface_pitch_a = param & 0xFFFF;
            ch->d3d_surface_pitch_z = param >> 16;
            break;
        case 0x0310:  // Set surface offset
            ch->d3d_surface_color_offset = param;
            break;
        case 0x0314:  // Set depth offset
            ch->d3d_surface_zeta_offset = param;
            break;
            
        // Viewport
        case 0x0400:  // Viewport horizontal
            ch->d3d_viewport_horizontal = param;
            break;
        case 0x0404:  // Viewport vertical  
            ch->d3d_viewport_vertical = param;
            break;
        case 0x0408:  // Viewport offset
            ch->d3d_viewport_offset[0] = u.param_float;
            break;
        case 0x040C:
            ch->d3d_viewport_offset[1] = u.param_float;
            break;
        case 0x0410:
            ch->d3d_viewport_offset[2] = u.param_float;
            break;
        case 0x0414:
            ch->d3d_viewport_offset[3] = u.param_float;
            break;
        case 0x0418:  // Viewport scale
            ch->d3d_viewport_scale[0] = u.param_float;
            break;
        case 0x041C:
            ch->d3d_viewport_scale[1] = u.param_float;
            break;
        case 0x0420:
            ch->d3d_viewport_scale[2] = u.param_float;
            break;
        case 0x0424:
            ch->d3d_viewport_scale[3] = u.param_float;
            break;
            
        // Depth and stencil
        case 0x0500:  // Depth test enable
            ch->d3d_depth_test_enable = param;
            break;
        case 0x0504:  // Depth write enable
            ch->d3d_depth_write_enable = param;
            break;
        case 0x0508:  // Depth function
            ch->d3d_depth_func = param;
            break;
        case 0x050C:  // Depth clip min
            ch->d3d_clip_min = u.param_float;
            break;
        case 0x0510:  // Depth clip max
            ch->d3d_clip_max = u.param_float;
            break;
            
        // Blending
        case 0x0600:  // Blend enable
            ch->d3d_blend_enable = param;
            break;
        case 0x0604:  // Blend source factor
            ch->d3d_blend_func_sfactor = param;
            break;
        case 0x0608:  // Blend dest factor
            ch->d3d_blend_func_dfactor = param;
            break;
            
        // Culling
        case 0x0700:  // Cull face enable
            ch->d3d_cull_face_enable = param;
            break;
        case 0x0704:  // Cull face
            ch->d3d_cull_face = param;
            break;
        case 0x0708:  // Front face
            ch->d3d_front_face = param;
            break;
            
        // Lighting
        case 0x0800:  // Lighting enable
            ch->d3d_lighting_enable = param;
            break;
        case 0x0804:  // Light enable mask
            ch->d3d_light_enable_mask = param;
            break;
        case 0x0808:  // Shade mode
            ch->d3d_shade_mode = param;
            break;
        case 0x080C:  // Scene ambient color
            ch->d3d_scene_ambient_color[0] = u.param_float;
            break;
        case 0x0810:
            ch->d3d_scene_ambient_color[1] = u.param_float;
            break;
        case 0x0814:
            ch->d3d_scene_ambient_color[2] = u.param_float;
            break;
        case 0x0818:
            ch->d3d_scene_ambient_color[3] = u.param_float;
            break;
            
        // Textures
        case 0x1000:  // Texture 0 offset
            ch->d3d_textures[0].offset = param;
            break;
        case 0x1004:  // Texture 0 format
            ch->d3d_textures[0].format = param;
            ch->d3d_textures[0].width = 1 << ((param >> 20) & 0x0F);
            ch->d3d_textures[0].height = 1 << ((param >> 24) & 0x0F);
            ch->d3d_textures[0].pitch = ch->d3d_textures[0].width * 4;  // Assume 32-bit
            break;
        case 0x1008:  // Texture 0 address mode
            ch->d3d_textures[0].address_mode = param;
            break;
        case 0x100C:  // Texture 0 control0
            ch->d3d_textures[0].control0 = param;
            break;
        case 0x1010:  // Texture 0 control1  
            ch->d3d_textures[0].control1 = param;
            break;
        case 0x1014:  // Texture 0 filter
            ch->d3d_textures[0].filter = param;
            break;
            
        // Clear
        case 0x1D94:  // Color clear value
            ch->d3d_color_clear_value = param;
            break;
        case 0x1D98:  // Depth/stencil clear value
            ch->d3d_zstencil_clear_value = param;
            break;
        case 0x1D9C:  // Clear surface
            ch->d3d_clear_surface = param;
            if (param) {
                geforce_d3d_clear_surface(gdev, ch);
            }
            break;
            
        // Vertex arrays
        case 0x1400:  // Vertex array 0 offset
            ch->d3d_vertex_data_array_offset[0] = param;
            break;
        case 0x1404:  // Vertex array 0 format
            ch->d3d_vertex_data_array_format_type[0] = param & 0x0F;
            ch->d3d_vertex_data_array_format_size[0] = (param >> 4) & 0x0F;
            ch->d3d_vertex_data_array_format_stride[0] = (param >> 8) & 0xFF;
            break;
            
        // Primitive drawing
        case 0x1800:  // Begin/End
            ch->d3d_begin_end = param;
            if (param == 0) {
                // End primitive - process accumulated vertices
                if (ch->d3d_vertex_index >= 3) {
                    struct vertex_data vertices[3];
                    int v;
                    
                    // Convert accumulated vertex data to vertex structures
                    for (v = 0; v < 3; v++) {
                        vertices[v].position[0] = ch->d3d_vertex_data[v][0][0];
                        vertices[v].position[1] = ch->d3d_vertex_data[v][0][1];
                        vertices[v].position[2] = ch->d3d_vertex_data[v][0][2];
                        vertices[v].position[3] = ch->d3d_vertex_data[v][0][3];
                        
                        vertices[v].color[0] = ch->d3d_vertex_data[v][3][0];
                        vertices[v].color[1] = ch->d3d_vertex_data[v][3][1];
                        vertices[v].color[2] = ch->d3d_vertex_data[v][3][2];
                        vertices[v].color[3] = ch->d3d_vertex_data[v][3][3];
                    }
                    
                    // Rasterize triangle
                    geforce_d3d_rasterize_triangle(gdev, ch, &vertices[0], &vertices[1], &vertices[2]);
                    
                    printk(KERN_INFO "GeForce: Rendered triangle\n");
                }
                ch->d3d_vertex_index = 0;
            }
            break;
            
        // Vertex attributes
        case 0x1500:  // Vertex position X
            if (ch->d3d_vertex_index < 4) {
                ch->d3d_vertex_data[ch->d3d_vertex_index][0][0] = u.param_float;
            }
            break;
        case 0x1504:  // Vertex position Y
            if (ch->d3d_vertex_index < 4) {
                ch->d3d_vertex_data[ch->d3d_vertex_index][0][1] = u.param_float;
            }
            break;
        case 0x1508:  // Vertex position Z
            if (ch->d3d_vertex_index < 4) {
                ch->d3d_vertex_data[ch->d3d_vertex_index][0][2] = u.param_float;
                ch->d3d_vertex_data[ch->d3d_vertex_index][0][3] = 1.0f;
                ch->d3d_vertex_index++;
            }
            break;
        case 0x1510:  // Vertex color R
            if (ch->d3d_vertex_index > 0) {
                ch->d3d_vertex_data[ch->d3d_vertex_index - 1][3][0] = u.param_float;
            }
            break;
        case 0x1514:  // Vertex color G
            if (ch->d3d_vertex_index > 0) {
                ch->d3d_vertex_data[ch->d3d_vertex_index - 1][3][1] = u.param_float;
            }
            break;
        case 0x1518:  // Vertex color B
            if (ch->d3d_vertex_index > 0) {
                ch->d3d_vertex_data[ch->d3d_vertex_index - 1][3][2] = u.param_float;
            }
            break;
        case 0x151C:  // Vertex color A
            if (ch->d3d_vertex_index > 0) {
                ch->d3d_vertex_data[ch->d3d_vertex_index - 1][3][3] = u.param_float;
            }
            break;
            
        default:
            printk(KERN_DEBUG "GeForce: Unknown D3D method 0x%04x\n", method);
            break;
    }
}

// IRQ level calculation (from Bochs)
static u32 geforce_get_mc_intr(struct virtual_geforce_dev *gdev)
{
    u32 value = 0x00000000;
    
    if (gdev->bus_intr & gdev->bus_intr_en)
        value |= 0x10000000;
    if (gdev->fifo_intr & gdev->fifo_intr_en)
        value |= 0x00000100;
    if (gdev->graph_intr & gdev->graph_intr_en)
        value |= 0x00001000;
    if (gdev->crtc_intr & gdev->crtc_intr_en)
        value |= 0x01000000;
        
    return value;
}

// IRQ handler
static irqreturn_t geforce_irq_handler(int irq, void *dev_id)
{
    struct virtual_geforce_dev *gdev = dev_id;
    u32 intr_status;
    unsigned long flags;
    
    spin_lock_irqsave(&gdev->reg_lock, flags);
    
    intr_status = geforce_get_mc_intr(gdev);
    if (!intr_status || !(gdev->mc_intr_en & 1)) {
        spin_unlock_irqrestore(&gdev->reg_lock, flags);
        return IRQ_NONE;
    }
    
    // Handle interrupts
    if (intr_status & 0x00000100) { // FIFO interrupt
        queue_work(gdev->workqueue, &gdev->fifo_work);
    }
    
    if (intr_status & 0x00001000) { // Graphics interrupt
        queue_work(gdev->workqueue, &gdev->d3d_work);
        printk(KERN_DEBUG "GeForce: Graphics interrupt - D3D pipeline\n");
    }
    
    if (intr_status & 0x01000000) { // CRTC interrupt
        // Handle CRTC interrupt (vertical retrace, etc.)
        printk(KERN_DEBUG "GeForce: CRTC interrupt\n");
    }
    
    spin_unlock_irqrestore(&gdev->reg_lock, flags);
    return IRQ_HANDLED;
}

// CRTC register access (adapted from Bochs)
static u8 geforce_crtc_read(struct virtual_geforce_dev *gdev, u32 index)
{
    unsigned long flags;
    u8 value;
    
    spin_lock_irqsave(&gdev->reg_lock, flags);
    
    if (index <= GEFORCE_CRTC_MAX) {
        value = gdev->crtc.reg[index];
        printk(KERN_DEBUG "GeForce: CRTC read index 0x%02x = 0x%02x\n", index, value);
    } else {
        printk(KERN_WARNING "GeForce: CRTC read unknown index 0x%02x\n", index);
        value = 0xff;
    }
    
    spin_unlock_irqrestore(&gdev->reg_lock, flags);
    return value;
}

static void geforce_crtc_write(struct virtual_geforce_dev *gdev, u32 index, u8 value)
{
    unsigned long flags;
    bool update_cursor_addr = false;
    
    spin_lock_irqsave(&gdev->reg_lock, flags);
    
    printk(KERN_DEBUG "GeForce: CRTC write index 0x%02x = 0x%02x\n", index, value);
    
    // Handle specific CRTC register updates (from Bochs)
    if (index == 0x1c) {
        if (!(gdev->crtc.reg[index] & 0x80) && (value & 0x80) != 0) {
            // Without clearing this register, Windows 95 hangs after reboot
            gdev->crtc_intr_en = 0x00000000;
        }
    } else if (index == 0x2f || index == 0x30 || index == 0x31) {
        update_cursor_addr = true;
    } else if (index == 0x25 || index == 0x2D || index == 0x41 || index == 0x42) {
        // These registers affect display mode
        gdev->svga_needs_update_mode = true;
    }
    
    if (index <= GEFORCE_CRTC_MAX) {
        gdev->crtc.reg[index] = value;
    } else {
        printk(KERN_WARNING "GeForce: CRTC write unknown index 0x%02x\n", index);
    }
    
    // Update cursor address if needed
    if (update_cursor_addr) {
        gdev->crtc_cursor_offset =
            (gdev->crtc.reg[0x31] >> 2 << 11) |
            (gdev->crtc.reg[0x30] & 0x7F) << 17 |
            gdev->crtc.reg[0x2f] << 24;
        gdev->crtc_cursor_offset += gdev->crtc_cursor_offset;
        printk(KERN_DEBUG "GeForce: Cursor offset updated to 0x%08x\n", gdev->crtc_cursor_offset);
    }
    
    // Update display mode if needed
    if (gdev->svga_needs_update_mode) {
        // Recalculate display parameters (simplified from Bochs)
        u32 iTopOffset = gdev->crtc.reg[0x0d] |
                        (gdev->crtc.reg[0x0c] << 8) |
                        (gdev->crtc.reg[0x19] & 0x1F) << 16;
        iTopOffset <<= 2;
        iTopOffset += gdev->crtc_start;
        
        u32 iPitch = gdev->crtc.reg[0x13] |
                    (gdev->crtc.reg[0x19] >> 5 << 8) |
                    (gdev->crtc.reg[0x42] >> 6 & 1) << 11;
        iPitch <<= 3;
        
        u32 iWidth = (gdev->crtc.reg[1] +
                     ((gdev->crtc.reg[0x2D] & 0x02) << 7) + 1) * 8;
        u32 iHeight = (gdev->crtc.reg[18] |
                      ((gdev->crtc.reg[7] & 0x02) << 7) |
                      ((gdev->crtc.reg[7] & 0x40) << 3) |
                      ((gdev->crtc.reg[0x25] & 0x02) << 9) |
                      ((gdev->crtc.reg[0x41] & 0x04) << 9)) + 1;
        
        gdev->svga_xres = iWidth;
        gdev->svga_yres = iHeight;
        gdev->svga_pitch = iPitch;
        
        printk(KERN_INFO "GeForce: Display mode updated to %dx%d, pitch %d\n", 
               iWidth, iHeight, iPitch);
        
        gdev->svga_needs_update_mode = false;
    }
    
    spin_unlock_irqrestore(&gdev->reg_lock, flags);
}

// FIFO Command Processing (enhanced with 3D support)
static bool geforce_execute_command(struct virtual_geforce_dev *gdev, u32 chid, u32 subc, u32 method, u32 param)
{
    struct geforce_channel *ch = &gdev->channels[chid];
    u32 object = ch->subchannel[subc].object;
    u32 class_id = object & gdev->class_mask;
    
    printk(KERN_DEBUG "GeForce: Execute command ch=%d subc=%d obj=0x%08x method=0x%04x param=0x%08x\n",
           chid, subc, object, method, param);
    
    // Route to appropriate handler based on object class
    switch (class_id) {
        case 0x004A:  // GDI Rectangle
        case 0x005F:  // Image Blit
        case 0x009F:  // Image from CPU
            // 2D operations - simplified handlers
            printk(KERN_DEBUG "GeForce: 2D operation class=0x%04x\n", class_id);
            break;
            
        case 0x0097:  // Kelvin (GeForce3) 3D class
        case 0x0497:  // Curie (GeForce6800) 3D class  
        case 0x0397:  // NV30/NV40 3D class
            // 3D operations
            geforce_execute_d3d_command(gdev, ch, class_id, method, param);
            gdev->d3d_pipeline_active = true;
            gdev->active_3d_channel = chid;
            break;
            
        case 0x0039:  // M2MF (Memory to Memory Format)
            // Memory copy operations
            printk(KERN_DEBUG "GeForce: M2MF operation method=0x%04x\n", method);
            break;
            
        default:
            printk(KERN_DEBUG "GeForce: Unknown object class 0x%04x\n", class_id);
            break;
    }
    
    return true;
}

// FIFO Processing (adapted from Bochs fifo_process)
static void geforce_fifo_process(struct virtual_geforce_dev *gdev, u32 chid)
{
    struct geforce_channel *ch = &gdev->channels[chid];
    u32 get = gdev->fifo_cache1_get / 4;
    u32 put = gdev->fifo_cache1_put / 4;
    
    while (get != put) {
        u32 method = gdev->fifo_cache1_method[get];
        u32 data = gdev->fifo_cache1_data[get];
        u32 subc = (method >> 13) & 7;
        u32 mthd = method & 0x1FFF;
        
        if (!geforce_execute_command(gdev, chid, subc, mthd, data)) {
            printk(KERN_ERR "GeForce: Command execution failed\n");
            break;
        }
        
        get = (get + 1) % GEFORCE_CACHE1_SIZE;
    }
    
    gdev->fifo_cache1_get = get * 4;
}

// Register read handler (enhanced with 3D registers)
static u32 geforce_register_read32(struct virtual_geforce_dev *gdev, u32 address)
{
    u32 value = 0;
    unsigned long flags;
    
    spin_lock_irqsave(&gdev->reg_lock, flags);
    
    if (address == 0x0) {
        // Device identification
        if (gdev->card_type == GEFORCE_3)
            value = 0x020200A5;
        else
            value = gdev->card_type << 20;
    } else if (address == 0x100) {
        value = geforce_get_mc_intr(gdev);
    } else if (address == 0x140) {
        value = gdev->mc_intr_en;
    } else if (address == 0x200) {
        value = gdev->mc_enable;
    } else if (address == 0x1100) {
        value = gdev->bus_intr;
    } else if (address == 0x1140) {
        value = gdev->bus_intr_en;
    } else if (address == 0x2100) {
        value = gdev->fifo_intr;
    } else if (address == 0x2140) {
        value = gdev->fifo_intr_en;
    } else if (address == 0x2210) {
        value = gdev->fifo_ramht;
    } else if (address == 0x2214 && gdev->card_type < GEFORCE_6800) {
        value = gdev->fifo_ramfc;
    } else if (address == 0x2218) {
        value = gdev->fifo_ramro;
    } else if (address == 0x2220 && gdev->card_type >= GEFORCE_6800) {
        value = gdev->fifo_ramfc;
    } else if (address == 0x2400) { // PFIFO_RUNOUT_STATUS
        value = 0x00000010;
        if (gdev->fifo_cache1_get != gdev->fifo_cache1_put)
            value = 0x00000000;
    } else if (address == 0x2504) {
        value = gdev->fifo_mode;
    } else if (address == 0x3204) {
        value = gdev->fifo_cache1_push1;
    } else if (address == 0x3210) {
        value = gdev->fifo_cache1_put;
    } else if (address == 0x3214) { // PFIFO_CACHE1_STATUS
        value = 0x00000010;
        if (gdev->fifo_cache1_get != gdev->fifo_cache1_put)
            value = 0x00000000;
    } else if (address == 0x3220) {
        value = gdev->fifo_cache1_dma_push;
    } else if (address == 0x322c) {
        value = gdev->fifo_cache1_dma_instance;
    } else if (address == 0x3230) { // PFIFO_CACHE1_DMA_CTL
        value = 0x80000000;
    } else if (address == 0x3240) {
        value = gdev->fifo_cache1_dma_put;
    } else if (address == 0x3244) {
        value = gdev->fifo_cache1_dma_get;
    } else if (address == 0x3248) {
        value = gdev->fifo_cache1_ref_cnt;
    } else if (address == 0x3250) {
        if (gdev->fifo_cache1_get != gdev->fifo_cache1_put)
            gdev->fifo_cache1_pull0 |= 0x00000100;
        value = gdev->fifo_cache1_pull0;
    } else if (address == 0x3270) {
        value = gdev->fifo_cache1_get;
    } else if (address == 0x32e0) {
        value = gdev->fifo_grctx_instance;
    } else if (address == 0x10020c) {
        value = gdev->vram_size;
    } else if (address == 0x100320) { // PFB_ZCOMP_SIZE  
        if (gdev->card_type == GEFORCE_3)
            value = 0x00007fff;
        else if (gdev->card_type == GEFORCE_FX_5900)
            value = 0x0005c7ff;
        else
            value = 0x0002e3ff;
    } else if (address == 0x101000) {
        value = gdev->straps0_primary;
    } else if (address == 0x400100) {
        value = gdev->graph_intr;
    } else if (address == 0x400108) {
        value = gdev->graph_nsource;
    } else if ((address == 0x40013C && gdev->card_type >= GEFORCE_6800) ||
               (address == 0x400140 && gdev->card_type < GEFORCE_6800)) {
        value = gdev->graph_intr_en;
    } else if (address == 0x40014C) {
        value = gdev->graph_ctx_switch1;
    } else if (address == 0x400150) {
        value = gdev->graph_ctx_switch2;
    } else if (address == 0x400158) {
        value = gdev->graph_ctx_switch4;
    } else if (address == 0x40032c) {
        value = gdev->graph_ctxctl_cur;
    } else if (address == 0x400700) {
        value = gdev->graph_status;
    } else if (address == 0x400704) {
        value = gdev->graph_trapped_addr;
    } else if (address == 0x400708) {
        value = gdev->graph_trapped_data;
    } else if (address == 0x400718) {
        value = gdev->graph_notify;
    } else if (address == 0x400720) {
        value = gdev->graph_fifo;
    } else if (address == 0x400780) {
        value = gdev->graph_channel_ctx_table;
    } else if (address == 0x400820 && gdev->card_type == GEFORCE_3) {
        value = gdev->graph_offset0;
    } else if (address == 0x400850 && gdev->card_type == GEFORCE_3) {
        value = gdev->graph_pitch0;
    } else if (address == 0x600100) {
        value = gdev->crtc_intr;
    } else if (address == 0x600140) {
        value = gdev->crtc_intr_en;
    } else if (address == 0x600800) {
        value = gdev->crtc_start;
    } else if (address == 0x600804) {
        value = gdev->crtc_config;
    } else if (address == 0x60080c) {
        value = gdev->crtc_cursor_offset;
    } else if (address == 0x600810) {
        value = gdev->crtc_cursor_config;
    } else if (address == 0x680300) {
        value = gdev->ramdac_cu_start_pos;
    } else if (address == 0x680404) { // RAMDAC_NV10_CURSYNC
        value = 0x00000000;
    } else if (address == 0x680508) {
        value = gdev->ramdac_vpll;
    } else if (address == 0x68050c) {
        value = gdev->ramdac_pll_select;
    } else if (address == 0x680578) {
        value = gdev->ramdac_vpll_b;
    } else if (address == 0x680600) {
        value = gdev->ramdac_general_control;
    } else if (address >= 0x700000 && address < 0x800000) {
        u32 offset = address & 0x000fffff;
        if (offset & 3) {
            value = geforce_ramin_read32(gdev, offset & ~3);
        } else {
            value = geforce_ramin_read32(gdev, offset);
        }
    } else {
        // Unknown register - return 0 or implement as needed
        printk(KERN_DEBUG "GeForce: Unknown register read 0x%08x\n", address);
        value = 0;
    }
    
    spin_unlock_irqrestore(&gdev->reg_lock, flags);
    return value;
}

// Register write handler (enhanced with 3D pipeline support)
static void geforce_register_write32(struct virtual_geforce_dev *gdev, u32 address, u32 value)
{
    unsigned long flags;
    
    spin_lock_irqsave(&gdev->reg_lock, flags);
    
    if (address == 0x140) {
        gdev->mc_intr_en = value;
    } else if (address == 0x200) {
        gdev->mc_enable = value;
    } else if (address == 0x1100) {
        gdev->bus_intr &= ~value;
    } else if (address == 0x1140) {
        gdev->bus_intr_en = value;
    } else if (address == 0x2100) {
        gdev->fifo_intr &= ~value;
    } else if (address == 0x2140) {
        gdev->fifo_intr_en = value;
    } else if (address == 0x2210) {
        gdev->fifo_ramht = value;
    } else if (address == 0x2214 && gdev->card_type < GEFORCE_6800) {
        gdev->fifo_ramfc = value;
    } else if (address == 0x2218) {
        gdev->fifo_ramro = value;
    } else if (address == 0x2220 && gdev->card_type >= GEFORCE_6800) {
        gdev->fifo_ramfc = value;
    } else if (address == 0x2504) {
        gdev->fifo_mode = value;
    } else if (address == 0x3204) {
        gdev->fifo_cache1_push1 = value;
    } else if (address == 0x3210) {
        gdev->fifo_cache1_put = value;
        // Trigger FIFO processing when PUT pointer is updated
        if (value != gdev->fifo_cache1_get) {
            queue_work(gdev->workqueue, &gdev->fifo_work);
        }
    } else if (address == 0x3220) {
        gdev->fifo_cache1_dma_push = value;
    } else if (address == 0x322c) {
        gdev->fifo_cache1_dma_instance = value;
    } else if (address == 0x3240) {
        gdev->fifo_cache1_dma_put = value;
    } else if (address == 0x3244) {
        gdev->fifo_cache1_dma_get = value;
    } else if (address == 0x3248) {
        gdev->fifo_cache1_ref_cnt = value;
    } else if (address == 0x3250) {
        gdev->fifo_cache1_pull0 = value;
    } else if (address == 0x3270) {
        gdev->fifo_cache1_get = value & (GEFORCE_CACHE1_SIZE * 4 - 1);
        if (gdev->fifo_cache1_get != gdev->fifo_cache1_put) {
            gdev->fifo_intr |= 0x00000001;
        } else {
            gdev->fifo_intr &= ~0x00000001;
            gdev->fifo_cache1_pull0 &= ~0x00000100;
        }
    } else if (address == 0x32e0) {
        gdev->fifo_grctx_instance = value;
    } else if (address == 0x101000) {
        if (value >> 31)
            gdev->straps0_primary = value;
        else
            gdev->straps0_primary = gdev->straps0_primary_original;
    } else if (address == 0x400100) {
        gdev->graph_intr &= ~value;
    } else if (address == 0x400108) {
        gdev->graph_nsource = value;
    } else if ((address == 0x40013C && gdev->card_type >= GEFORCE_6800) ||
               (address == 0x400140 && gdev->card_type < GEFORCE_6800)) {
        gdev->graph_intr_en = value;
    } else if (address == 0x40014C) {
        gdev->graph_ctx_switch1 = value;
    } else if (address == 0x400150) {
        gdev->graph_ctx_switch2 = value;
    } else if (address == 0x400158) {
        gdev->graph_ctx_switch4 = value;
    } else if (address == 0x40032c) {
        gdev->graph_ctxctl_cur = value;
    } else if (address == 0x400700) {
        gdev->graph_status = value;
    } else if (address == 0x400704) {
        gdev->graph_trapped_addr = value;
    } else if (address == 0x400708) {
        gdev->graph_trapped_data = value;
    } else if (address == 0x400718) {
        gdev->graph_notify = value;
    } else if (address == 0x400720) {
        gdev->graph_fifo = value;
    } else if (address == 0x400780) {
        gdev->graph_channel_ctx_table = value;
    } else if (address == 0x400820 && gdev->card_type == GEFORCE_3) {
        gdev->graph_offset0 = value;
    } else if (address == 0x400850 && gdev->card_type == GEFORCE_3) {
        gdev->graph_pitch0 = value;
    } else if (address == 0x600100) {
        gdev->crtc_intr &= ~value;
    } else if (address == 0x600140) {
        gdev->crtc_intr_en = value;
    } else if (address == 0x600800) {
        gdev->crtc_start = value;
        gdev->svga_needs_update_mode = true;
    } else if (address == 0x600804) {
        gdev->crtc_config = value;
    } else if (address == 0x60080c) {
        gdev->crtc_cursor_offset = value;
    } else if (address == 0x600810) {
        gdev->crtc_cursor_config = value;
    } else if (address == 0x680300) {
        gdev->ramdac_cu_start_pos = value;
    } else if (address == 0x680508) {
        gdev->ramdac_vpll = value;
    } else if (address == 0x68050c) {
        gdev->ramdac_pll_select = value;
    } else if (address == 0x680578) {
        gdev->ramdac_vpll_b = value;
    } else if (address == 0x680600) {
        gdev->ramdac_general_control = value;
    } else if (address >= 0x700000 && address < 0x800000) {
        geforce_ramin_write32(gdev, address - 0x700000, value);
    } else {
        // Unknown register write
        printk(KERN_DEBUG "GeForce: Unknown register write 0x%08x = 0x%08x\n", address, value);
    }
    
    spin_unlock_irqrestore(&gdev->reg_lock, flags);
}

// FIFO work handler (enhanced for 3D pipeline)
static void geforce_fifo_work(struct work_struct *work)
{
    struct virtual_geforce_dev *gdev = container_of(work, struct virtual_geforce_dev, fifo_work);
    
    // Process FIFO commands for all channels
    int chid;
    for (chid = 0; chid < GEFORCE_CHANNEL_COUNT; chid++) {
        geforce_fifo_process(gdev, chid);
    }
    
    printk(KERN_DEBUG "GeForce: FIFO processing complete\n");
}

// 3D Pipeline work handler
static void geforce_d3d_work(struct work_struct *work)
{
    struct virtual_geforce_dev *gdev = container_of(work, struct virtual_geforce_dev, d3d_work);
    
    mutex_lock(&gdev->d3d_mutex);
    
    if (gdev->d3d_pipeline_active && gdev->active_3d_channel < GEFORCE_CHANNEL_COUNT) {
        struct geforce_channel *ch = &gdev->channels[gdev->active_3d_channel];
        
        printk(KERN_DEBUG "GeForce: Processing 3D pipeline for channel %d\n", gdev->active_3d_channel);
        
        // Process any pending 3D operations
        // This would normally involve vertex processing, rasterization, etc.
        // For now, just log the activity
        
        // Reset pipeline state after processing
        gdev->d3d_pipeline_active = false;
    }
    
    mutex_unlock(&gdev->d3d_mutex);
}

// PCI configuration space access - Fixed to return proper PCIBIOS codes
static int geforce_pci_read_config(struct pci_bus *bus, unsigned int devfn, 
                                   int where, int size, u32 *val)
{
    struct virtual_geforce_dev *gdev = global_geforce_dev;
    
    if (!gdev || !gdev->pci_dev) {
        *val = 0xFFFFFFFF;
        return PCIBIOS_DEVICE_NOT_FOUND;
    }
    
    // Handle basic PCI config reads
    switch (where) {
        case PCI_VENDOR_ID:
            *val = (gdev->pci_dev->device << 16) | gdev->pci_dev->vendor;
            break;
        case PCI_COMMAND:
            *val = PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER;
            break;
        case PCI_STATUS:
            *val = PCI_STATUS_CAP_LIST;
            break;
        case PCI_CLASS_REVISION:
            *val = (PCI_CLASS_DISPLAY_VGA << 16) | 0x00;
            break;
        case PCI_BASE_ADDRESS_0:
            *val = gdev->mmio_base ? (u32)(uintptr_t)gdev->mmio_base : 0;
            break;
        case PCI_BASE_ADDRESS_1:
            *val = gdev->vram_dma_addr ? (u32)gdev->vram_dma_addr : 0;
            break;
        case PCI_INTERRUPT_LINE:
            *val = gdev->irq_line;
            break;
        default:
            *val = 0x00000000;
            break;
    }
    
    return PCIBIOS_SUCCESSFUL;
}

static int geforce_pci_write_config(struct pci_bus *bus, unsigned int devfn, 
                                    int where, int size, u32 val)
{
    struct virtual_geforce_dev *gdev = global_geforce_dev;
    
    if (!gdev || !gdev->pci_dev) {
        return PCIBIOS_DEVICE_NOT_FOUND;
    }
    
    // Handle PCI config writes (mostly ignored for virtual device)
    printk(KERN_DEBUG "GeForce: PCI config write offset 0x%02x = 0x%08x\n", where, val);
    
    return PCIBIOS_SUCCESSFUL;
}

static struct pci_ops geforce_pci_ops = {
    .read = geforce_pci_read_config,
    .write = geforce_pci_write_config,
};

// Memory access handlers
static int geforce_mmio_mmap(struct file *filp, struct vm_area_struct *vma)
{
    struct virtual_geforce_dev *gdev = filp->private_data;
    unsigned long size = vma->vm_end - vma->vm_start;
    
    if (size > GEFORCE_PNPMMIO_SIZE)
        return -EINVAL;
    
    vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
    
    if (remap_pfn_range(vma, vma->vm_start,
                        virt_to_phys(gdev->mmio_base) >> PAGE_SHIFT,
                        size, vma->vm_page_prot))
        return -EAGAIN;
    
    return 0;
}

static long geforce_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct virtual_geforce_dev *gdev = filp->private_data;
    
    switch (cmd) {
        case 0x1000: // Read register
            {
                u32 __user *user_arg = (u32 __user *)arg;
                u32 offset, value;
                
                if (get_user(offset, user_arg))
                    return -EFAULT;
                
                if (offset >= GEFORCE_PNPMMIO_SIZE)
                    return -EINVAL;
                
                value = geforce_register_read32(gdev, offset);
                
                if (put_user(value, user_arg + 1))
                    return -EFAULT;
                    
                return 0;
            }
        case 0x1001: // Write register
            {
                u32 __user *user_arg = (u32 __user *)arg;
                u32 offset, value;
                
                if (get_user(offset, user_arg) || get_user(value, user_arg + 1))
                    return -EFAULT;
                
                if (offset >= GEFORCE_PNPMMIO_SIZE)
                    return -EINVAL;
                
                geforce_register_write32(gdev, offset, value);
                return 0;
            }
        case 0x1002: // Read CRTC register
            {
                u32 __user *user_arg = (u32 __user *)arg;
                u32 index;
                u8 value;
                
                if (get_user(index, user_arg))
                    return -EFAULT;
                
                if (index > GEFORCE_CRTC_MAX)
                    return -EINVAL;
                
                value = geforce_crtc_read(gdev, index);
                
                if (put_user((u32)value, user_arg + 1))
                    return -EFAULT;
                    
                return 0;
            }
        case 0x1003: // Write CRTC register
            {
                u32 __user *user_arg = (u32 __user *)arg;
                u32 index, value;
                
                if (get_user(index, user_arg) || get_user(value, user_arg + 1))
                    return -EFAULT;
                
                if (index > GEFORCE_CRTC_MAX)
                    return -EINVAL;
                
                geforce_crtc_write(gdev, index, (u8)value);
                return 0;
            }
        case 0x1004: // Get device info
            {
                struct {
                    u32 card_type;
                    u32 vram_size;
                    u32 xres, yres, bpp;
                    bool d3d_active;
                } info;
                
                info.card_type = gdev->card_type;
                info.vram_size = gdev->vram_size;
                info.xres = gdev->svga_xres;
                info.yres = gdev->svga_yres;
                info.bpp = gdev->svga_bpp;
                info.d3d_active = gdev->d3d_pipeline_active;
                
                if (copy_to_user((void __user *)arg, &info, sizeof(info)))
                    return -EFAULT;
                    
                return 0;
            }
        case 0x1005: // Execute 3D command
            {
                struct {
                    u32 channel;
                    u32 subchannel;
                    u32 method;
                    u32 param;
                } cmd;
                
                if (copy_from_user(&cmd, (void __user *)arg, sizeof(cmd)))
                    return -EFAULT;
                
                if (cmd.channel >= GEFORCE_CHANNEL_COUNT || cmd.subchannel >= GEFORCE_SUBCHANNEL_COUNT)
                    return -EINVAL;
                
                geforce_execute_command(gdev, cmd.channel, cmd.subchannel, cmd.method, cmd.param);
                return 0;
            }
        default:
            return -ENOTTY;
    }
}

static int geforce_open(struct inode *inode, struct file *filp)
{
    struct virtual_geforce_dev *gdev = global_geforce_dev;
    
    if (!gdev)
        return -ENODEV;
    
    filp->private_data = gdev;
    return 0;
}

static int geforce_release(struct inode *inode, struct file *filp)
{
    return 0;
}

static struct file_operations geforce_fops = {
    .owner = THIS_MODULE,
    .open = geforce_open,  
    .release = geforce_release,
    .unlocked_ioctl = geforce_ioctl,
    .mmap = geforce_mmio_mmap,
};

static struct miscdevice geforce_misc_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "geforce_emu",
    .fops = &geforce_fops,
};

// Initialize GeForce device state (enhanced with 3D pipeline)
static void geforce_init_state(struct virtual_geforce_dev *gdev)
{
    int i, j;
    
    // Clear CRTC registers
    gdev->crtc.index = GEFORCE_CRTC_MAX + 1;
    for (i = 0; i <= GEFORCE_CRTC_MAX; i++)
        gdev->crtc.reg[i] = 0x00;
    
    // Initialize interrupt state
    gdev->mc_intr_en = 0;
    gdev->mc_enable = 0;
    gdev->bus_intr = 0;
    gdev->bus_intr_en = 0;
    gdev->fifo_intr = 0;
    gdev->fifo_intr_en = 0;
    
    // Initialize FIFO state
    gdev->fifo_ramht = 0;
    gdev->fifo_ramfc = 0;
    gdev->fifo_ramro = 0;
    gdev->fifo_mode = 0;
    gdev->fifo_cache1_push1 = 0;
    gdev->fifo_cache1_put = 0;
    gdev->fifo_cache1_dma_push = 0;
    gdev->fifo_cache1_dma_instance = 0;
    gdev->fifo_cache1_dma_put = 0;
    gdev->fifo_cache1_dma_get = 0;
    gdev->fifo_cache1_ref_cnt = 0;
    gdev->fifo_cache1_pull0 = 0;
    gdev->fifo_cache1_semaphore = 0;
    gdev->fifo_cache1_get = 0;
    gdev->fifo_grctx_instance = 0;
    
    for (i = 0; i < GEFORCE_CACHE1_SIZE; i++) {
        gdev->fifo_cache1_method[i] = 0;
        gdev->fifo_cache1_data[i] = 0;
    }
    
    // Initialize graphics engine state
    gdev->graph_intr = 0;
    gdev->graph_nsource = 0;
    gdev->graph_intr_en = 0;
    gdev->graph_ctx_switch1 = 0;
    gdev->graph_ctx_switch2 = 0;
    gdev->graph_ctx_switch4 = 0;
    gdev->graph_ctxctl_cur = 0;
    gdev->graph_status = 0;
    gdev->graph_trapped_addr = 0;
    gdev->graph_trapped_data = 0;
    gdev->graph_notify = 0;
    gdev->graph_fifo = 0;
    gdev->graph_channel_ctx_table = 0;
    gdev->graph_offset0 = 0;
    gdev->graph_pitch0 = 0;
    
    // Initialize CRTC state
    gdev->crtc_intr = 0;
    gdev->crtc_intr_en = 0;
    gdev->crtc_start = 0;
    gdev->crtc_config = 0;
    gdev->crtc_cursor_offset = 0;
    gdev->crtc_cursor_config = 0;
    
    // Initialize RAMDAC state
    gdev->ramdac_cu_start_pos = 0;
    gdev->ramdac_vpll = 0;
    gdev->ramdac_vpll_b = 0;
    gdev->ramdac_pll_select = 0;
    gdev->ramdac_general_control = 0;
    
    // Initialize display state
    gdev->svga_xres = 640;
    gdev->svga_yres = 480;
    gdev->svga_bpp = 8;
    gdev->svga_pitch = 640;
    gdev->svga_needs_update_mode = false;
    
    // Initialize channels with 3D support
    for (i = 0; i < GEFORCE_CHANNEL_COUNT; i++) {
        struct geforce_channel *ch = &gdev->channels[i];
        memset(ch, 0, sizeof(*ch));
        
        // Initialize 3D pipeline state for each channel
        ch->d3d_viewport_horizontal = 640;
        ch->d3d_viewport_vertical = 480;
        ch->d3d_surface_format = 0x00000055;  // Default to A8R8G8B8 + Z24S8
        ch->d3d_color_bytes = 4;
        ch->d3d_depth_bytes = 4;
        ch->d3d_depth_func = D3D_DEPTH_LESS;
        ch->d3d_blend_func_sfactor = D3D_BLEND_ONE;
        ch->d3d_blend_func_dfactor = D3D_BLEND_ZERO;
        
        // Initialize lights
        for (j = 0; j < MAX_LIGHTS; j++) {
            ch->d3d_lights[j].diffuse_color[0] = 1.0f;
            ch->d3d_lights[j].diffuse_color[1] = 1.0f;
            ch->d3d_lights[j].diffuse_color[2] = 1.0f;
            ch->d3d_lights[j].infinite_direction[0] = 0.0f;
            ch->d3d_lights[j].infinite_direction[1] = 0.0f;
            ch->d3d_lights[j].infinite_direction[2] = -1.0f;  // Point down
            ch->d3d_lights[j].enabled = false;
        }
        
        // Initialize transformation matrices to identity
        for (j = 0; j < 16; j++) {
            ch->d3d_composite_matrix[j] = (j % 5 == 0) ? 1.0f : 0.0f;
        }
        
        // Initialize viewport transform
        ch->d3d_viewport_scale[0] = 320.0f;   // width/2
        ch->d3d_viewport_scale[1] = -240.0f;  // -height/2 (flip Y)
        ch->d3d_viewport_scale[2] = 0.5f;     // depth scale
        ch->d3d_viewport_scale[3] = 0.0f;
        
        ch->d3d_viewport_offset[0] = 320.0f;  // width/2
        ch->d3d_viewport_offset[1] = 240.0f;  // height/2
        ch->d3d_viewport_offset[2] = 0.5f;    // depth offset
        ch->d3d_viewport_offset[3] = 0.0f;
    }
    
    // Initialize 3D pipeline state
    gdev->d3d_pipeline_active = false;
    gdev->active_3d_channel = 0;
    
    // Set up card-specific parameters (from Bochs)
    if (gdev->card_type == GEFORCE_3) {
        gdev->vram_size = GEFORCE_VRAM_SIZE_64MB;
        gdev->bar2_size = 0x00080000;
        // Matches real hardware with exception of disabled TV out
        gdev->straps0_primary_original = (0x7FF86C6B | 0x00000180);
    } else if (gdev->card_type == GEFORCE_FX_5900) {
        gdev->vram_size = GEFORCE_VRAM_SIZE_128MB;
        gdev->bar2_size = 0x01000000;
        // Guess
        gdev->straps0_primary_original = (0x7FF86C4B | 0x00000180);
    } else { // GEFORCE_6800
        gdev->vram_size = GEFORCE_VRAM_SIZE_256MB;
        gdev->bar2_size = 0x01000000;
        // Guess
        gdev->straps0_primary_original = (0x7FF86C4B | 0x00000180);
    }
    
    gdev->straps0_primary = gdev->straps0_primary_original;
    gdev->ramin_flip = gdev->vram_size - 64;
    gdev->memsize_mask = gdev->vram_size - 1;
    gdev->class_mask = gdev->card_type < GEFORCE_6800 ? 0x00000FFF : 0x0000FFFF;
}

static int __init virtual_geforce_init(void)
{
    struct virtual_geforce_dev *gdev;
    int ret;
    static struct list_head virtual_resources = LIST_HEAD_INIT(virtual_resources);
    
    printk(KERN_INFO "Virtual GeForce PCI: Initializing module with 3D pipeline support\n");
    
    // Allocate device structure
    gdev = kzalloc(sizeof(*gdev), GFP_KERNEL);
    if (!gdev)
        return -ENOMEM;
    
    global_geforce_dev = gdev;
    
    // Set default card type (can be made module parameter)
    gdev->card_type = GEFORCE_3;
    
    // Initialize device state
    geforce_init_state(gdev);
    
    // Initialize synchronization primitives
    spin_lock_init(&gdev->reg_lock);
    mutex_init(&gdev->d3d_mutex);
    
    // Create workqueue for FIFO and 3D processing
    gdev->workqueue = create_singlethread_workqueue("geforce_wq");
    if (!gdev->workqueue) {
        ret = -ENOMEM;
        goto err_free_dev;
    }
    
    INIT_WORK(&gdev->fifo_work, geforce_fifo_work);
    INIT_WORK(&gdev->d3d_work, geforce_d3d_work);
    
    // Allocate VRAM (DMA coherent memory)
    gdev->vram_base = dma_alloc_coherent(NULL, gdev->vram_size, 
                                        &gdev->vram_dma_addr, GFP_KERNEL);
    if (!gdev->vram_base) {
        printk(KERN_ERR "Virtual GeForce PCI: Failed to allocate VRAM\n");
        ret = -ENOMEM;
        goto err_destroy_wq;
    }
    
    // Clear VRAM
    memset(gdev->vram_base, 0, gdev->vram_size);
    
    // Allocate MMIO region
    gdev->mmio_base = ioremap(0, GEFORCE_PNPMMIO_SIZE);
    if (!gdev->mmio_base) {
        // For virtual device, we can simulate this without real MMIO
        printk(KERN_WARNING "Virtual GeForce PCI: MMIO allocation failed, using simulation\n");
    }
    
    // Create dummy parent device
    gdev->parent_dev = kzalloc(sizeof(struct device), GFP_KERNEL);
    if (!gdev->parent_dev) {
        ret = -ENOMEM;
        goto err_free_mmio;
    }
    
    device_initialize(gdev->parent_dev);
    dev_set_name(gdev->parent_dev, "virtual_geforce_parent");
    
    // Create virtual PCI bus
    gdev->virtual_bus = pci_create_root_bus(gdev->parent_dev, 0, &geforce_pci_ops, 
                                           NULL, &virtual_resources);
    if (!gdev->virtual_bus) {
        printk(KERN_ERR "Virtual GeForce PCI: Failed to create virtual PCI bus\n");
        ret = -ENOMEM;
        goto err_free_parent;
    }
    
    // Allocate virtual PCI device
    gdev->pci_dev = pci_alloc_dev(gdev->virtual_bus);
    if (!gdev->pci_dev) {
        printk(KERN_ERR "Virtual GeForce PCI: Failed to allocate virtual PCI device\n");
        ret = -ENOMEM;
        goto err_remove_bus;
    }
    
    // Initialize PCI device properties
    gdev->pci_dev->bus = gdev->virtual_bus;
    gdev->pci_dev->sysdata = gdev->virtual_bus->sysdata;
    gdev->pci_dev->dev.parent = gdev->virtual_bus->bridge;
    gdev->pci_dev->devfn = PCI_DEVFN(0, 0);  // Slot 0, function 0
    
    // Set device IDs based on card type
    gdev->pci_dev->vendor = GEFORCE_VENDOR_ID;
    if (gdev->card_type == GEFORCE_3) {
        gdev->pci_dev->device = GEFORCE_DEVICE_ID_GF3;
    } else if (gdev->card_type == GEFORCE_FX_5900) {
        gdev->pci_dev->device = GEFORCE_DEVICE_ID_FX5900;
    } else {
        gdev->pci_dev->device = GEFORCE_DEVICE_ID_6800;
    }
    
    gdev->pci_dev->class = (PCI_CLASS_DISPLAY_VGA << 8) | 0x00;
    gdev->pci_dev->subsystem_vendor = GEFORCE_VENDOR_ID;
    gdev->pci_dev->subsystem_device = gdev->pci_dev->device;
    gdev->pci_dev->hdr_type = PCI_HEADER_TYPE_NORMAL;
    gdev->pci_dev->multifunction = 0;
    
    // Request IRQ
    gdev->irq_line = 11; // Use IRQ 11 for virtual device
    ret = request_irq(gdev->irq_line, geforce_irq_handler, IRQF_SHARED, 
                      "virtual_geforce", gdev);
    if (ret) {
        printk(KERN_WARNING "Virtual GeForce PCI: Failed to request IRQ %d\n", gdev->irq_line);
        gdev->irq_line = 0;
    }
    
    // Add device to bus
    pci_device_add(gdev->pci_dev, gdev->pci_dev->bus);
    pci_bus_add_device(gdev->pci_dev);
    
    // Rescan to update sysfs and lspci
    pci_bus_assign_resources(gdev->virtual_bus);
    pci_bus_add_devices(gdev->virtual_bus);
    pci_scan_child_bus(gdev->virtual_bus);
    
    // Register misc device for userspace access
    ret = misc_register(&geforce_misc_device);
    if (ret) {
        printk(KERN_ERR "Virtual GeForce PCI: Failed to register misc device\n");
        goto err_free_irq;
    }
    
    printk(KERN_INFO "Virtual GeForce PCI: Device created successfully\n");
    printk(KERN_INFO "  Card Type: %s (0x%02x)\n", 
           gdev->card_type == GEFORCE_3 ? "GeForce3 Ti 500" :
           gdev->card_type == GEFORCE_FX_5900 ? "GeForce FX 5900" : "GeForce 6800 GT",
           gdev->card_type);
    printk(KERN_INFO "  VRAM Size: %d MB\n", gdev->vram_size / (1024 * 1024));
    printk(KERN_INFO "  3D Pipeline: Enabled\n");
    printk(KERN_INFO "  Bus: %d, Device: %d:%d\n",
           gdev->pci_dev->bus->number, 
           PCI_SLOT(gdev->pci_dev->devfn), 
           PCI_FUNC(gdev->pci_dev->devfn));
    
    return 0;
    
err_free_irq:
    if (gdev->irq_line)
        free_irq(gdev->irq_line, gdev);
    pci_stop_and_remove_bus_device(gdev->pci_dev);
    pci_dev_put(gdev->pci_dev);
err_remove_bus:
    pci_remove_bus(gdev->virtual_bus);
err_free_parent:
    put_device(gdev->parent_dev);
    kfree(gdev->parent_dev);
err_free_mmio:
    if (gdev->mmio_base)
        iounmap(gdev->mmio_base);
    dma_free_coherent(NULL, gdev->vram_size, gdev->vram_base, gdev->vram_dma_addr);
err_destroy_wq:
    destroy_workqueue(gdev->workqueue);
err_free_dev:
    kfree(gdev);
    global_geforce_dev = NULL;
    return ret;
}

static void __exit virtual_geforce_exit(void)
{
    struct virtual_geforce_dev *gdev = global_geforce_dev;
    
    if (!gdev)
        return;
    
    printk(KERN_INFO "Virtual GeForce PCI: Unloading module\n");
    
    // Unregister misc device
    misc_deregister(&geforce_misc_device);
    
    // Free IRQ
    if (gdev->irq_line)
        free_irq(gdev->irq_line, gdev);
    
    // Clean up workqueue
    if (gdev->workqueue) {
        flush_workqueue(gdev->workqueue);
        destroy_workqueue(gdev->workqueue);
    }
    
    // Remove PCI device
    if (gdev->pci_dev) {
        pci_stop_and_remove_bus_device(gdev->pci_dev);
        pci_dev_put(gdev->pci_dev);
    }
    
    // Remove PCI bus
    if (gdev->virtual_bus)
        pci_remove_bus(gdev->virtual_bus);
    
    // Clean up parent device
    if (gdev->parent_dev) {
        put_device(gdev->parent_dev);
        kfree(gdev->parent_dev);
    }
    
    // Free memory regions
    if (gdev->mmio_base)
        iounmap(gdev->mmio_base);
    
    if (gdev->vram_base)
        dma_free_coherent(NULL, gdev->vram_size, gdev->vram_base, gdev->vram_dma_addr);
    
    // Free device structure
    kfree(gdev);
    global_geforce_dev = NULL;
    
    printk(KERN_INFO "Virtual GeForce PCI: Module unloaded\n");
}

module_init(virtual_geforce_init);
module_exit(virtual_geforce_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Based on Bochs GeForce emulation");
MODULE_DESCRIPTION("Virtual GeForce PCI Device with 3D Pipeline and VFIO/IOMMU support");
MODULE_VERSION("1.1");

// Module parameters
static int card_type = GEFORCE_3;
module_param(card_type, int, 0644);
MODULE_PARM_DESC(card_type, "GeForce card type (0x20=GF3, 0x35=FX5900, 0x40=6800)");

static bool enable_3d = true;
module_param(enable_3d, bool, 0644);
MODULE_PARM_DESC(enable_3d, "Enable 3D pipeline emulation");
