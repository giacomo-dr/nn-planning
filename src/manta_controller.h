// manta_controller.h
// Author: Giacomo Del Rio
// Date: 12 Apr 2017

#ifndef MANTA_CONTROLLER_H
#define MANTA_CONTROLLER_H

#include "vrep_client.h"


class MantaController {
public:
    MantaController( VRepClient& vrep );
    ~MantaController(){};
    VRepClient& get_client() const;
    void set_velocity( double mt_sec );
    void set_steer( double angle );
    void set_position( double x, double y, double z );
    void set_orientation( double a, double b, double g );
    void set_pose( Point3D pos, Point3D orient );
    void register_for_getting_pose( int delay_ms );
    void unregister_for_getting_pose();
    Point3D get_position();
    Point3D get_orientation();

private:
    VRepClient& vrep;
    int motor_handle;
    int steer_handle;
    int base_handle;
};


#endif //MANTA_CONTROLLER_H
