# Blender Unbender

A Blender add-on that helps you unfold 3D models into flat patterns using animated shape keys. Perfect for creating papercraft models, origami designs, or any project requiring 3D to 2D unfolding.

## Features

- **Edge Marking System**
  - **Precut**: Mark edges to be cut before processing
  - **Freeze**: Lock edge angles in place during unbending
  - **Freestyle**: Mark edges for cosmetic/visual purposes

- **Multiple Unfolding Methods**
  - **By Path**: Unfold each path independently
  - **Trunk First**: Unfold from origin outward to branches
  - **Branches First**: Unfold from outer branches inward to trunk

- **Cleanup Options**
  - Keep all geometry intact
  - Remove edge markings only
  - Remove all faces (leaving wireframe)
  - Remove all fold edges except frozen ones
  - Remove all fold edges (except Freestyle marks)

- **Animation Control**
  - Shape key-based animation
  - Individual path control
  - Global unfolding control

## Requirements

- Blender 4.1 or newer
- Windows/Mac/Linux

## Installation

1. Download `unbender.zip`
2. Open Blender
3. Go to Edit > Preferences > Add-ons
4. Click "Install..."
5. Navigate to and select the downloaded `unbender.zip` file
6. Enable the add-on by checking its checkbox

## Usage

1. **Access the Tool**
   - Open the sidebar in the 3D View (press N)
   - Find the "Unbender" tab

2. **Step-by-Step Workflow**
   1. Select a single mesh object in the 3D view
   2. Switch to Edit Mode (Tab key)
   3. Mark edges as needed:
      - Use "Precut" for edges you want to separate
      - Use "Freeze" for edges that should maintain their angle
      - Use "Freestyle" for visual line markings
   4. Switch back to Object Mode (Tab key)
   5. Choose your unbending method and post-processing option
   6. Click "Unbend"
   7. A new object will be created with:
      - Shape keys for animation control
      - Custom driver sliders for fine-tuning

3. **Edge Marking Tips**
   - Precut edges will be split before processing
   - Frozen edges maintain their angles during unbending
   - Freestyle marks are purely visual
   - Note: Freeze takes precedence over Precut if both are set

4. **Face Normals**
   - If faces appear inside-out, use the "Flip Face" tool
   - Select faces in Edit Mode to flip their normals
   - Tip: Flipped faces will fold inward, opposite direction of adjacent faces, but require a small non-flat starting angle 


## Tutorial Videos (TBD)

## Known Issues

- Large models may take longer to process
- Complex geometry might require manual face normal adjustments
-- if your object has non-flat n-gon faces, unfolding won't work
- Flipped faces will fold inward, but require a slight non-180 degree starting angle 

## Contributing

Feel free to submit issues and enhancement requests!

## License

GPL-3.0

## Credits

Created by Chris Allen

## Version History

- 1.0.0: Initial release
  - Basic unfolding functionality
  - Three unfolding methods
  - Edge marking system
  - Cleanup options
