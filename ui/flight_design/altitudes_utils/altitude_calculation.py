def calculate_altitude(ui):
    """Calculate altitude ASL and AGL.

    The flight altitude is referenced to the lowest terrain elevation of the
    flight area, so the specified GSD is achieved there and bettered everywhere
    else.
    """
    gsd = ui.doubleSpinBoxGSD.value() / 100
    min_h = ui.doubleSpinBoxMinHeight.value()

    if ui.radioButtonGSD.isChecked():
        altitude_AGL = gsd / ui.camera_handler.camera.sensor_size * ui.camera_handler.camera.focal_length
    elif ui.radioButtonAltAGL.isChecked():
        altitude_AGL = ui.doubleSpinBoxAltAGL.value()

    ui.progressBar.setValue(20)
    return min_h + altitude_AGL, altitude_AGL