#include <iostream>



int main()
{
    double num_pol = 20;

    for (int i=0; i<num_pol; i++)
    {   std::cout
        << "    - Alpha: 0.05000000074505806\n"
        << "      BoundColor: 0; 0; 0\n" 
        << "      Class: decomp_rviz_plugins/PolyhedronArray\n"
        << "      Enabled: true \n"
        << "      MeshColor: 46; 52; 54 \n"
        << "      Name: Obstacles" << i << "\n"
        << "      Queue Size: 10\n"
        << "      Scale: 0.009999999776482582\n"
        << "      State: Both\n"
        << "      Topic: poly_jps_int_" << i << "\n"
        << "      Unreliable: false\n"
        << "      Value: true\n"
        << "      VsColor: 239; 41; 41\n"
        << "      VsScale: 1"
        << std::endl;
    }
}

