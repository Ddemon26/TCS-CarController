
# TCS Car Controller

[![Unity Version](https://img.shields.io/badge/unity-2020.3%2B-blue.svg)](https://unity.com/)
[![License](https://img.shields.io/github/license/Ddemon26/TCS-CarController)](./LICENSE)
[![GitHub contributors](https://img.shields.io/github/contributors/Ddemon26/TCS-CarController)](https://github.com/Ddemon26/TCS-CarController/graphs/contributors)
[![Issues](https://img.shields.io/github/issues/Ddemon26/TCS-CarController)](https://github.com/Ddemon26/TCS-CarController/issues)

## Overview

The **TCS Car Controller** is a customizable Unity-based system designed for simulating realistic vehicle physics, driving control, and visual effects in a game or simulation environment. This system provides developers with the flexibility to integrate different types of drive mechanisms (such as front-wheel, rear-wheel, and all-wheel drive), manage speed units (MPH or KPH), and simulate real-time car dynamics and effects, such as wheel spinning and engine sounds.

### Key Features:
- **Car Drive Types**: Supports All-Wheel Drive, Front-Wheel Drive, and Rear-Wheel Drive modes.
- **Speed Units**: Option to use Miles per hour (MPH) or Kilometers per hour (KPH).
- **Wheel Effects**: Visual effects such as tire smoke.
- **Light Simulation**: Brake and reverse lights.
- **Customizable Car Components**: Configure wheels, audio, and physics properties.

## Requirements

- Unity 2020.3 or later.
- Requires the Unity `Rigidbody` component for handling physics.

## Installation

1. Clone the repository:
    ```bash
    git clone https://github.com/Ddemon26/TCS-CarController.git
    ```
2. Open the project in Unity and ensure that you have the necessary dependencies installed.

## Basic Setup

1. Add the `CarController` script to a car GameObject in your Unity scene.
2. Assign the required components such as:
   - Wheel Meshes
   - Wheel Colliders
   - Wheel Effects
3. Set the drive type and speed unit according to your needs:
    ```csharp
    m_carDriveType = CarDriveType.AllWheelDrive;
    m_speedType = SpeedType.Mph;
    ```

## Usage Example

Below is a simplified example to showcase the basic implementation of the `CarController` system in a Unity scene:

```csharp
using TCS.CarController;
using UnityEngine;

public class CarSetup : MonoBehaviour
{
    public CarController carController;

    void Start()
    {
        // Setting the drive type to All-Wheel Drive
        carController.m_carDriveType = CarDriveType.AllWheelDrive;
        
        // Setting the speed type to MPH
        carController.m_speedType = SpeedType.Mph;

        // Initialize other car components here
    }

    void Update()
    {
        // You can update car settings dynamically if needed
    }
}
```

## Contributing

Contributions are welcome! Please follow these steps to contribute:
1. Fork the repo.
2. Create a feature branch (`git checkout -b feature-branch-name`).
3. Commit your changes (`git commit -m 'Add feature'`).
4. Push to the branch (`git push origin feature-branch-name`).
5. Create a new Pull Request.

## License

This project is licensed under the MIT License - see the [LICENSE](./LICENSE) file for details.

## Contact

For any inquiries or issues, feel free to reach out to the repository owner or create an issue [here](https://github.com/Ddemon26/TCS-CarController/issues).
