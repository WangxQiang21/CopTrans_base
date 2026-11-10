# Formation-reference primitive, NOT a per-vehicle A/O/S mode primitive.
uint8 TRANSLATION=0
uint8 ROTATION=1
uint8 COMPOSITE=2
uint8 geometry_type
float64 amount       # translation: m; rotation: rad
float64 control      # first version leaves semantic detail to the geometry contract
