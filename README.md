# Library Serial Port POSIX (libserialposix)

Is a lightweight and portable C and C++ library providing a simple API for serial port communication on POSIX compliant systems. 

Designed specifically for robotics and embedded systems, it prioritizes a small footprint and avoids extra dependencies to streamline the installation process.  
Leveraging versatile I/O and low-level control, it offers a straightforward way to read and write data using familiar standard library functions.  

This library is intended for users familiar with their hardware configurations, and therefore does not include features such as automatic port discovery or baud rate detection. 
 
This library is intended for communication with non-canonical devices.

---

## Table of Contents

- [Features](#features)
- [Building](#building)
- [Installation](#installation)
- [Examples](#examples)
- [License](#license)
- [Contact](#contact)

---

## Features

*   **No External Dependencies:**  Easy to integrate into embedded systems and projects with minimal overhead.
*   **Configurable:** Supports setting baud rate, parity, stop bits and flow control.
*   **Deterministic:** Supports timeouts or minimum number of bytes to block read functions.
*   **Documentation:** Documentation done in doxygen.
*   **Tested:** Includes a suite of unit tests on real hardware.

---

## Examples



---

## License

This project is licensed under the [GNU_V3 License](./../../LICENSE).

---

## Contact

For any questions or issues, please reach out via email at pacheco.castro.fabio@gmail.com or fabio.d.pacheco@inesctec.pt.