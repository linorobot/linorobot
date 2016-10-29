// This header is meant to undo the effect of having included
// usb_desc.h.  Everything usb_desc.h, usb_undef.h undefines.
// This stuff is not supposed to be made visible to user level
// programs, but it's nice if we can use it within the headers
// included by user programs, to automatically define only the
// appropriate APIs for the types of USB interfaces used.

#ifdef _usb_desc_h_
#undef _usb_desc_h_
#endif
#ifdef ENDPOINT_UNUSED
#undef ENDPOINT_UNUSED
#endif
#ifdef ENDPOINT_TRANSIMIT_ONLY
#undef ENDPOINT_TRANSIMIT_ONLY
#endif
#ifdef ENDPOINT_RECEIVE_ONLY
#undef ENDPOINT_RECEIVE_ONLY
#endif
#ifdef ENDPOINT_TRANSMIT_AND_RECEIVE
#undef ENDPOINT_TRANSMIT_AND_RECEIVE
#endif
#ifdef VENDOR_ID
#undef VENDOR_ID
#endif
#ifdef PRODUCT_ID
#undef PRODUCT_ID
#endif
#ifdef DEVICE_CLASS
#undef DEVICE_CLASS
#endif
#ifdef MANUFACTURER_NAME
#undef MANUFACTURER_NAME
#endif
#ifdef MANUFACTURER_NAME_LEN
#undef MANUFACTURER_NAME_LEN
#endif
#ifdef PRODUCT_NAME
#undef PRODUCT_NAME 
#endif
#ifdef PRODUCT_NAME_LEN
#undef PRODUCT_NAME_LEN
#endif
#ifdef EP0_SIZE
#undef EP0_SIZE
#endif
#ifdef NUM_ENDPOINTS
#undef NUM_ENDPOINTS
#endif
#ifdef NUM_USB_BUFFERS
#undef NUM_USB_BUFFERS
#endif
#ifdef NUM_INTERFACE
#undef NUM_INTERFACE
#endif
#ifdef CDC_STATUS_INTERFACE
#undef CDC_STATUS_INTERFACE
#endif
#ifdef CDC_DATA_INTERFACE
#undef CDC_DATA_INTERFACE
#endif
#ifdef CDC_ACM_ENDPOINT
#undef CDC_ACM_ENDPOINT
#endif
#ifdef CDC_RX_ENDPOINT
#undef CDC_RX_ENDPOINT
#endif
#ifdef CDC_TX_ENDPOINT
#undef CDC_TX_ENDPOINT
#endif
#ifdef CDC_ACM_SIZE
#undef CDC_ACM_SIZE
#endif
#ifdef CDC_RX_SIZE
#undef CDC_RX_SIZE
#endif
#ifdef CDC_TX_SIZE
#undef CDC_TX_SIZE
#endif
#ifdef SEREMU_INTERFACE
#undef SEREMU_INTERFACE
#endif
#ifdef SEREMU_TX_ENDPOINT
#undef SEREMU_TX_ENDPOINT
#endif
#ifdef SEREMU_TX_SIZE
#undef SEREMU_TX_SIZE
#endif
#ifdef SEREMU_TX_INTERVAL
#undef SEREMU_TX_INTERVAL
#endif
#ifdef SEREMU_RX_ENDPOINT
#undef SEREMU_RX_ENDPOINT
#endif
#ifdef SEREMU_RX_SIZE
#undef SEREMU_RX_SIZE
#endif
#ifdef SEREMU_RX_INTERVAL
#undef SEREMU_RX_INTERVAL
#endif
#ifdef KEYBOARD_INTERFACE
#undef KEYBOARD_INTERFACE
#endif
#ifdef KEYBOARD_ENDPOINT
#undef KEYBOARD_ENDPOINT
#endif
#ifdef KEYBOARD_SIZE
#undef KEYBOARD_SIZE
#endif
#ifdef KEYBOARD_INTERVAL
#undef KEYBOARD_INTERVAL
#endif
#ifdef MOUSE_INTERFACE
#undef MOUSE_INTERFACE
#endif
#ifdef MOUSE_ENDPOINT
#undef MOUSE_ENDPOINT
#endif
#ifdef MOUSE_SIZE
#undef MOUSE_SIZE
#endif
#ifdef MOUSE_INTERVAL
#undef MOUSE_INTERVAL
#endif
#ifdef JOYSTICK_INTERFACE
#undef JOYSTICK_INTERFACE
#endif
#ifdef JOYSTICK_ENDPOINT
#undef JOYSTICK_ENDPOINT
#endif
#ifdef JOYSTICK_SIZE
#undef JOYSTICK_SIZE
#endif
#ifdef JOYSTICK_INTERVAL
#undef JOYSTICK_INTERVAL
#endif
#ifdef CDC_IAD_DESCRIPTOR
#undef CDC_IAD_DESCRIPTOR
#endif
#ifdef MIDI_INTERFACE
#undef MIDI_TX_ENDPOINT
#endif
#ifdef MIDI_TX_SIZE
#undef MIDI_TX_SIZE
#endif
#ifdef MIDI_RX_ENDPOINT
#undef MIDI_RX_ENDPOINT
#endif
#ifdef MIDI_RX_SIZE
#undef MIDI_RX_SIZE
#endif
#ifdef RAWHID_INTERFACE
#undef RAWHID_INTERFACE
#endif
#ifdef RAWHID_TX_ENDPOINT
#undef RAWHID_TX_ENDPOINT
#endif
#ifdef RAWHID_TX_SIZE
#undef RAWHID_TX_SIZE
#endif
#ifdef RAWHID_TX_INTERVAL
#undef RAWHID_TX_INTERVAL
#endif
#ifdef RAWHID_RX_ENDPOINT
#undef RAWHID_RX_ENDPOINT
#endif
#ifdef RAWHID_RX_SIZE
#undef RAWHID_RX_SIZE
#endif
#ifdef RAWHID_RX_INTERVAL
#undef RAWHID_RX_INTERVAL
#endif
#ifdef FLIGHTSIM_INTERFACE
#undef FLIGHTSIM_TX_ENDPOINT
#endif
#ifdef FLIGHTSIM_TX_SIZE
#undef FLIGHTSIM_TX_SIZE
#endif
#ifdef FLIGHTSIM_TX_INTERVAL
#undef FLIGHTSIM_TX_INTERVAL
#endif
#ifdef FLIGHTSIM_RX_ENDPOINT
#undef FLIGHTSIM_RX_ENDPOINT
#endif
#ifdef FLIGHTSIM_RX_SIZE
#undef FLIGHTSIM_RX_SIZE
#endif
#ifdef MTP_INTERFACE
#undef MTP_INTERFACE
#endif
#ifdef MTP_TX_ENDPOINT
#undef MTP_TX_ENDPOINT
#endif
#ifdef MTP_TX_SIZE
#undef MTP_TX_SIZE
#endif
#ifdef MTP_RX_ENDPOINT
#undef MTP_RX_ENDPOINT
#endif
#ifdef MTP_RX_SIZE
#undef MTP_RX_SIZE
#endif
#ifdef MTP_EVENT_ENDPOINT
#undef MTP_EVENT_ENDPOINT
#endif
#ifdef MTP_EVENT_SIZE
#undef MTP_EVENT_SIZE
#endif
#ifdef MTP_EVENT_INTERVAL
#undef MTP_EVENT_INTERVAL
#endif
#ifdef ENDPOINT1_CONFIG
#undef ENDPOINT1_CONFIG
#endif
#ifdef ENDPOINT2_CONFIG
#undef ENDPOINT2_CONFIG
#endif
#ifdef ENDPOINT3_CONFIG
#undef ENDPOINT3_CONFIG
#endif
#ifdef ENDPOINT4_CONFIG
#undef ENDPOINT4_CONFIG
#endif
#ifdef ENDPOINT5_CONFIG
#undef ENDPOINT5_CONFIG
#endif
#ifdef ENDPOINT6_CONFIG
#undef ENDPOINT6_CONFIG
#endif
#ifdef ENDPOINT7_CONFIG
#undef ENDPOINT7_CONFIG
#endif
#ifdef ENDPOINT8_CONFIG
#undef ENDPOINT8_CONFIG
#endif
#ifdef ENDPOINT9_CONFIG
#undef ENDPOINT9_CONFIG
#endif
#ifdef ENDPOINT10_CONFIG
#undef ENDPOINT10_CONFIG
#endif
#ifdef ENDPOINT11_CONFIG
#undef ENDPOINT11_CONFIG
#endif
#ifdef ENDPOINT12_CONFIG
#undef ENDPOINT12_CONFIG
#endif
#ifdef ENDPOINT13_CONFIG
#undef ENDPOINT13_CONFIG
#endif
#ifdef ENDPOINT14_CONFIG
#undef ENDPOINT14_CONFIG
#endif
#ifdef ENDPOINT15_CONFIG
#undef ENDPOINT15_CONFIG
#endif

