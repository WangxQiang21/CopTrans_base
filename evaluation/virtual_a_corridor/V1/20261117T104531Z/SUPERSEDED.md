# SUPERSEDED: first V1 display attempt (20261117T104531Z)

This directory is retained unchanged for audit and is not the final V1 run.

The first real RViz display exposed a rendering defect not caught by the pure
marker tests: RViz multiplies `TRIANGLE_LIST` point coordinates by
`Marker.scale`, whose default is 0.1.  Every corridor fill was therefore
shrunk to 10 % around the map origin while the `LINE_STRIP` edges used the
original coordinates.  The code now sets the fill scale to `(1, 1, 1)` and
`tests/test_virtual_a_corridor_display.py` asserts it.

` screenshot_second.png ` was also invalid evidence: RViz appended `*` to its
window title after the view changed, the window lookup failed, and ImageMagick
captured the active editor window instead.  It was retaken against the RViz
window before this attempt was abandoned; both screenshots remain here only as
debugging evidence.  The final V1 run is a later UTC directory.
