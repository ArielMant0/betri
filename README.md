# betri
OpenFlipper-Plugin to create and display Bézier triangles

## Setup

- Make sure new Version of Qt is installed which includes **QT_NO_FLOAT16_OPERATORS**
    - can be added to cmake file with `add_compile_definitions(QT_NO_FLOAT16_OPERATORS)`
macro so there are no problems with the bitset header.
- Make sure this macro is defined in all relevant VS projects (OpenFlipper, OpenFlipperLib and all plugins)
- Make sure to use the correct cmake config for VS x64
