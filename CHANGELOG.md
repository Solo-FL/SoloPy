# Changelog 

All notable changes to this project will be documented in this file.

## [4.1.0] - 2025-03-21
### Added
- CANopen get_can_bus_baudrate

### Changed
- SOLOMotorControllersCanopen class to SoloMotorControllersCanopen

### Deprecated

### Removed
- UART serial_is_working() removed for communication_is_working()
### Fixed
- CANopen reset_position_to_zero() aligned to protocoll

## [4.0.0] - 2025-03-20
### Added
- Updated functions to match the latest firmware capabilities.
- Added CANopen Canable support for:
  - Windows
  - Linux
  - Raspberry Pi
- Added CANopen PDO capabilities for:
  - Windows
  - Linux
  - Raspberry Pi
- Restructured examples for:
  - Windows
  - Linux
  - Raspberry Pi
- Added README files for each example, providing relevant information about the section of the library.


### Changed
- Updated the main README.

### Deprecated

### Removed

### Fixed