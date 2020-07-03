<!--
*** Template source: https://github.com/othneildrew/Best-README-Template/blob/master/README.md
-->

<!-- PROJECT SHIELDS -->
<!--
*** I'm using markdown "reference style" links for readability.
*** Reference links are enclosed in brackets [ ] instead of parentheses ( ).
*** See the bottom of this document for the declaration of the reference variables
*** for contributors-url, forks-url, etc. This is an optional, concise syntax you may use.
*** https://www.markdownguide.org/basic-syntax/#reference-style-links
-->
[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]



<!-- PROJECT LOGO -->
<br />
<p align="center">
  <a href="https://github.com/tommasopolonelli/SynthSense-WSN-UAV">
    <img src="img/syslogo.jpg" alt="Logo" width="1269" height="325">
  </a>

  <h3 align="center">SynthSense-WSN-UAV</h3>

  <p align="center">
    A flexible, low-power platform for UAV-based data collection from remote sensors
    <br />
    <a href="https://github.com/tommasopolonelli/SynthSense-WSN-UAV/blob/master/README.md"><strong>Explore the docs »</strong></a>
    <br />
    <br />
    <a href="https://github.com/othneildrew/Best-README-Template">View Demo</a>
    ·
    <a href="https://github.com/tommasopolonelli/SynthSense-WSN-UAV/issues">Report Bug</a>
    ·
    <a href="https://github.com/tommasopolonelli/SynthSense-WSN-UAV/issues">Request Feature</a>
  </p>
</p>



<!-- TABLE OF CONTENTS -->
## Table of Contents

* [About the Project](#about-the-project)
  * [Built With](#built-with)
* [Getting Started](#getting-started)
  * [Prerequisites](#prerequisites)
  * [Installation](#installation)
* [Usage](#usage)
* [Roadmap](#roadmap)
* [Contributing](#contributing)
* [License](#license)
* [Contact](#contact)
* [Acknowledgements](#acknowledgements)



<!-- ABOUT THE PROJECT -->
## About The Project

This project holds the design and characterisation of a new low-power hardware platform to integrate unmanned aerial vehicle and wireless sensor technologies. In combination, these technologies can overcome data collection and maintenance problems of in situ monitoring in remote and extreme environments. Precision localisation in support of maximum efficiency mid-range inductive power transfer when recharging devices and increased throughput between drone and device are needed for data intensive monitoring applications, and to balance proximity time for devices powered by supercapacitors that recharge in seconds. The platform described in this paper incorporates ultra-wideband technology to achieve high-performance ranging and high data throughput. It enables the development of a new localisation system that is experimentally shown to improve accuracy by around two orders of magnitude to 10 cm with respect to GNSS and achieves almost 6 Mbps throughput in both lab and field conditions. These results are supported by extensive modelling and analysis. The platform is designed for application flexibility, and therefore includes a wide range of sensors and expansion possibilities, with source code for two applications made immediately available as part of a open source project to support research and development in this new area.

A list of commonly used resources that I find helpful are listed in the acknowledgements.

### Built With
This section should list any major frameworks that you built your project using. Leave any add-ons/plugins for the acknowledgements section. Here are a few examples.
* [Bootstrap](https://getbootstrap.com)
* [JQuery](https://jquery.com)
* [Laravel](https://laravel.com)



<!-- GETTING STARTED -->
## Getting Started

This is an example of how you may give instructions on setting up your project locally.
To get a local copy up and running follow these simple example steps.

### Prerequisites

This is an example of how to list things you need to use the software and how to install them.
* npm
```sh
npm install npm@latest -g
```

### Installation

1. Get a free API Key at [https://example.com](https://example.com)
2. Clone the repo
```sh
git clone https://github.com/your_username_/Project-Name.git
```
3. Install NPM packages
```sh
npm install
```
4. Enter your API in `config.js`
```JS
const API_KEY = 'ENTER YOUR API';
```



<!-- USAGE EXAMPLES -->
## Usage

[Work in progress](https://nebulab.it/blog/images/articles-images/maintenance-page-with-apache-and-capistrano/maintenance_page-12f699da.jpg).



<!-- ROADMAP -->
## Roadmap

See the [open issues](https://github.com/tommasopolonelli/SynthSense-WSN-UAV/issues) for a list of proposed features (and known issues).



<!-- CONTRIBUTING -->
## Contributing

Contributions are what make the open source community such an amazing place to be learn, inspire, and create. Any contributions you make are **greatly appreciated**.

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

If you use **SynthSense-WSN-UAV** in an academic or industrial context, please cite the following publications:

~~~~
@inproceedings{qin2019radio,
  title={Radio Diversity for Heterogeneous Communication with Wireless Sensors},
  author={Qin, Yuan and Boyle, David and Yeatman, Eric},
  booktitle={2019 IEEE 5th World Forum on Internet of Things (WF-IoT)},
  pages={955--960},
  year={2019},
  organization={IEEE}
}
~~~~

~~~~
@article{qin2019efficient,
  title={Efficient and reliable aerial communication with wireless sensors},
  author={Qin, Yuan and Boyle, David and Yeatman, Eric},
  journal={IEEE Internet of Things Journal},
  volume={6},
  number={5},
  pages={9000--9011},
  year={2019},
  publisher={IEEE}
}
~~~~

<!-- LICENSE -->
## License

Distributed under free License. See `LICENSE` for more information.



<!-- CONTACT -->
## Contact

David Boyle - david.boyle@imperial.ac.uk
Personal Web-Page: [LINK](https://www.imperial.ac.uk/people/david.boyle)


<!-- ACKNOWLEDGEMENTS -->
## Acknowledgements
* ["Efficient and reliable aerial communication with wireless sensors." IEEE Internet of Things Journal](https://ieeexplore.ieee.org/iel7/6488907/6702522/08752433.pdf)
* ["Radio Diversity for Heterogeneous Communication with Wireless Sensors." IEEE 5th World Forum on Internet of Things](https://ieeexplore.ieee.org/iel7/8764305/8767167/08767222.pdf)


<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/othneildrew/Best-README-Template.svg?style=flat-square
[contributors-url]: https://github.com/tommasopolonelli/SynthSense-WSN-UAV/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/othneildrew/Best-README-Template.svg?style=flat-square
[forks-url]: https://github.com/tommasopolonelli/SynthSense-WSN-UAV/network/members
[stars-shield]: https://img.shields.io/github/stars/othneildrew/Best-README-Template.svg?style=flat-square
[stars-url]: https://github.com/tommasopolonelli/SynthSense-WSN-UAV/stargazers
[issues-shield]: https://img.shields.io/github/issues/othneildrew/Best-README-Template.svg?style=flat-square
[issues-url]: https://github.com/tommasopolonelli/SynthSense-WSN-UAV/issues
[license-shield]: https://img.shields.io/github/license/othneildrew/Best-README-Template.svg?style=flat-square
[license-url]: https://github.com/tommasopolonelli/SynthSense-WSN-UAV/blob/master/LICENSE
[product-screenshot]: images/screenshot.png
