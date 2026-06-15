# Detekcja z drona — z telemetrią GPS i wysokości z mavros (USB → FCU)

std_msgs/Header header
uint32 object_id
string object_type
sensor_msgs/Image object_image
string qr_value

# Pozycja z /mavros/global_position/global
float64 latitude
float64 longitude
float64 altitude_gps

# Wysokość względna z /mavros/global_position/rel_alt (barometr FCU)
float64 altitude_baro

# Pozycja względna celu w układzie drona [m]: X=przód, Y=prawo, Z=w dół
float64 relative_x
float64 relative_y
float64 relative_z
