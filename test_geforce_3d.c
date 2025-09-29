#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <stdint.h>

// Test program for Virtual GeForce 3D functionality

#define IOCTL_READ_REG    0x1000
#define IOCTL_WRITE_REG   0x1001
#define IOCTL_GET_INFO    0x1004
#define IOCTL_EXEC_3D     0x1005

struct device_info {
    uint32_t card_type;
    uint32_t vram_size;
    uint32_t xres, yres, bpp;
    int d3d_active;
};

struct d3d_command {
    uint32_t channel;
    uint32_t subchannel;
    uint32_t method;
    uint32_t param;
};

int main(int argc, char *argv[])
{
    int fd;
    struct device_info info;
    struct d3d_command cmd;
    
    printf("Virtual GeForce 3D Pipeline Test\n");
    printf("================================\n\n");
    
    // Open device
    fd = open("/dev/geforce_emu", O_RDWR);
    if (fd < 0) {
        perror("Failed to open device");
        return 1;
    }
    
    // Get device information
    if (ioctl(fd, IOCTL_GET_INFO, &info) < 0) {
        perror("Failed to get device info");
        close(fd);
        return 1;
    }
    
    printf("Device Information:\n");
    printf("  Card Type: 0x%02x ", info.card_type);
    switch (info.card_type) {
        case 0x20: printf("(GeForce3 Ti 500)\n"); break;
        case 0x35: printf("(GeForce FX 5900)\n"); break;
        case 0x40: printf("(GeForce 6800 GT)\n"); break;
        default: printf("(Unknown)\n"); break;
    }
    printf("  VRAM Size: %d MB\n", info.vram_size / (1024 * 1024));
    printf("  Resolution: %dx%d @ %d bpp\n", info.xres, info.yres, info.bpp);
    printf("  3D Pipeline: %s\n\n", info.d3d_active ? "Active" : "Inactive");
    
    // Test 3D pipeline with a simple triangle
    printf("Testing 3D Pipeline...\n");
    
    // Set up 3D surface (640x480 ARGB8888)
    cmd.channel = 0;
    cmd.subchannel = 0;
    cmd.method = 0x0308;  // Surface format
    cmd.param = 0x00000055;  // A8R8G8B8 + Z24S8
    if (ioctl(fd, IOCTL_EXEC_3D, &cmd) < 0) {
        perror("Failed to set surface format");
    }
    
    // Set viewport
    cmd.method = 0x0400;  // Viewport horizontal
    cmd.param = 640;
    ioctl(fd, IOCTL_EXEC_3D, &cmd);
    
    cmd.method = 0x0404;  // Viewport vertical
    cmd.param = 480;
    ioctl(fd, IOCTL_EXEC_3D, &cmd);
    
    // Clear surface
    cmd.method = 0x1D94;  // Color clear value (blue)
    cmd.param = 0xFF0000FF;
    ioctl(fd, IOCTL_EXEC_3D, &cmd);
    
    cmd.method = 0x1D9C;  // Clear surface
    cmd.param = 0x01;  // Clear color buffer
    ioctl(fd, IOCTL_EXEC_3D, &cmd);
    
    // Begin triangle primitive
    cmd.method = 0x1800;  // Begin/End
    cmd.param = 0x0004;  // Triangle list
    ioctl(fd, IOCTL_EXEC_3D, &cmd);
    
    // Vertex 1 (top, red)
    cmd.method = 0x1500; cmd.param = *(uint32_t*)&(float){0.0f}; ioctl(fd, IOCTL_EXEC_3D, &cmd);  // X
    cmd.method = 0x1504; cmd.param = *(uint32_t*)&(float){-1.0f}; ioctl(fd, IOCTL_EXEC_3D, &cmd); // Y
    cmd.method = 0x1508; cmd.param = *(uint32_t*)&(float){0.0f}; ioctl(fd, IOCTL_EXEC_3D, &cmd);  // Z
    cmd.method = 0x1510; cmd.param = *(uint32_t*)&(float){1.0f}; ioctl(fd, IOCTL_EXEC_3D, &cmd);  // R
    cmd.method = 0x1514; cmd.param = *(uint32_t*)&(float){0.0f}; ioctl(fd, IOCTL_EXEC_3D, &cmd);  // G
    cmd.method = 0x1518; cmd.param = *(uint32_t*)&(float){0.0f}; ioctl(fd, IOCTL_EXEC_3D, &cmd);  // B
    cmd.method = 0x151C; cmd.param = *(uint32_t*)&(float){1.0f}; ioctl(fd, IOCTL_EXEC_3D, &cmd);  // A
    
    // Vertex 2 (bottom-left, green)  
    cmd.method = 0x1500; cmd.param = *(uint32_t*)&(float){-1.0f}; ioctl(fd, IOCTL_EXEC_3D, &cmd);
    cmd.method = 0x1504; cmd.param = *(uint32_t*)&(float){1.0f}; ioctl(fd, IOCTL_EXEC_3D, &cmd);
    cmd.method = 0x1508; cmd.param = *(uint32_t*)&(float){0.0f}; ioctl(fd, IOCTL_EXEC_3D, &cmd);
    cmd.method = 0x1510; cmd.param = *(uint32_t*)&(float){0.0f}; ioctl(fd, IOCTL_EXEC_3D, &cmd);
    cmd.method = 0x1514; cmd.param = *(uint32_t*)&(float){1.0f}; ioctl(fd, IOCTL_EXEC_3D, &cmd);
    cmd.method = 0x1518; cmd.param = *(uint32_t*)&(float){0.0f}; ioctl(fd, IOCTL_EXEC_3D, &cmd);
    cmd.method = 0x151C; cmd.param = *(uint32_t*)&(float){1.0f}; ioctl(fd, IOCTL_EXEC_3D, &cmd);
    
    // Vertex 3 (bottom-right, blue)
    cmd.method = 0x1500; cmd.param = *(uint32_t*)&(float){1.0f}; ioctl(fd, IOCTL_EXEC_3D, &cmd);
    cmd.method = 0x1504; cmd.param = *(uint32_t*)&(float){1.0f}; ioctl(fd, IOCTL_EXEC_3D, &cmd);
    cmd.method = 0x1508; cmd.param = *(uint32_t*)&(float){0.0f}; ioctl(fd, IOCTL_EXEC_3D, &cmd);
    cmd.method = 0x1510; cmd.param = *(uint32_t*)&(float){0.0f}; ioctl(fd, IOCTL_EXEC_3D, &cmd);
    cmd.method = 0x1514; cmd.param = *(uint32_t*)&(float){0.0f}; ioctl(fd, IOCTL_EXEC_3D, &cmd);
    cmd.method = 0x1518; cmd.param = *(uint32_t*)&(float){1.0f}; ioctl(fd, IOCTL_EXEC_3D, &cmd);
    cmd.method = 0x151C; cmd.param = *(uint32_t*)&(float){1.0f}; ioctl(fd, IOCTL_EXEC_3D, &cmd);
    
    // End primitive
    cmd.method = 0x1800;  // Begin/End
    cmd.param = 0x0000;   // End
    ioctl(fd, IOCTL_EXEC_3D, &cmd);
    
    printf("3D Triangle rendered!\n");
    printf("Check kernel messages with: dmesg | grep GeForce\n\n");
    
    close(fd);
    return 0;
}