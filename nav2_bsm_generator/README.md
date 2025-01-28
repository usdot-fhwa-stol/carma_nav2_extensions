# nav2_bsm_generator

The nav2_bsm_generator package contains a node that subscribes to various vehicle data topics in the CARMA System (including topics for vehicle speed, longitudinal acceleration, transmission state, and more). Using this received data, the BSM Generator node composes a BSM message and publishes the message at a fixed rate. This package is adapted from the [CARMA Platform implementation](https://github.com/usdot-fhwa-stol/carma-platform/tree/develop/bsm_generator).
