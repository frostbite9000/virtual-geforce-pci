#!/bin/bash
# Virtual GeForce PCI Driver Loading Script

# Default parameters
CARD_TYPE=${1:-0x20}  # GeForce3 by default
ENABLE_3D=${2:-1}     # Enable 3D pipeline by default

echo "Loading Virtual GeForce PCI Driver..."
echo "Card Type: $CARD_TYPE"
echo "3D Pipeline: $([ $ENABLE_3D -eq 1 ] && echo 'Enabled' || echo 'Disabled')"

# Load the module
if sudo insmod virtual_geforce_pci.ko card_type=$CARD_TYPE enable_3d=$ENABLE_3D; then
    echo "Driver loaded successfully!"
    
    # Wait a moment for device creation
    sleep 1
    
    # Show device information
    echo ""
    echo "Device Information:"
    ls -la /dev/geforce_emu 2>/dev/null || echo "Device node not created"
    
    echo ""
    echo "PCI Device:"
    lspci | grep -i nvidia || echo "Virtual device may not be visible in lspci yet"
    
    echo ""
    echo "Kernel Messages:"
    dmesg | tail -10 | grep -i geforce || echo "No recent GeForce messages"
    
else
    echo "Failed to load driver!"
    exit 1
fi