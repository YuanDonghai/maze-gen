
#include <iostream>
#include "path.h"

int main()
{
    Path test(60, 50);
    //test.show_matrix();
    test.set_start_end_point(30, 0, 30, 49);
    
    test.set_steps_range(100, 400);
    test.set_show_gui(600,400);

    test.rand_main_path();
    test.gen_matrix_wall_main_path();

    test.rand_dead_path();

    test.matrix_wall_re();
    test.matrix_wall_edge_zero();   
    test.matrix_wall_merge_area();

   
    test.show_gui_image(true,true);
}
