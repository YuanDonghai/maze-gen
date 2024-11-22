#pragma once
#include <iostream>
#include <random>
#include <vector>
#include <windows.h>


#include <opencv2/opencv.hpp>

using namespace std;

struct PathPoint {
    int x;
    int y;
    PathPoint& operator=(const PathPoint& other)
    {
        if (this != &other)
        {
            x = other.x;
            y = other.y;
        }
        return *this;
    }
    bool operator==(const PathPoint& other)
    {
        if (this != &other)
        {
            return (x == other.x) && (y == other.y);
        }
        return true;
    }
};
struct DirectAvaliable {
    int x_h = 1, x_l = 1;
    int y_h = 1, y_l = 1;
    int dir_counts = 4;
};

struct BigArea {
    struct PathPoint start_point;
    int width;
};

class Path {
public:
    Path(int x_max, int y_max);
    ~Path();
    /*
    set the start and end point. start with 0.

    it has correct position.
    - not the corner.
    - must on the edge.
    */
    void set_start_end_point(int start_x, int start_y, int end_x, int end_y);
    /*
    set the correct path steps range.
    without set ,the range will be
        min = 2 * (width + height),
        max = width * height / 4;
    */
    void set_steps_range(long step_min, long step_max);
    /*
    set the graphic result size.
    */
    void set_show_gui(int width, int height);
    /*
    set the deepth for generate child path and the water when child path break.
    - deep : int
    - water: int ,the percents
    */
    void set_deep_and_water(int deep, int water);
    /*
    generate the main path,which is the correct path.
    */
    void rand_main_path();
    /*
    generate the child path,which can not arrive the end.
    */
    void rand_dead_path();
    /*
    generate the matrix with walls,and set the main path point.
    */
    void gen_matrix_wall_main_path();
    /*
    re-set the matrix with walls data.
    */
    void matrix_wall_re();
    /*
    set the rectangle edge data as empty.
    */
    void matrix_wall_edge_zero();
    /*
    merge the nearby walls which is closed with others.
    - this make the gui result better.
    */
    void matrix_wall_merge_area();
    /*
    shwo the graphic result with picture,and save.

    - main_path_enable  : show the correct path with color-red.
    - other_path_enable : show all path with color-blue.
    - fname             : save the picture as fname-q.png for question,and fname-a.png for answer.
    */
    void show_gui_image(bool main_path_enable = false, bool other_path_enable = false, string fname = "m1");

    /*
    show base matrix.
    it print the result in console,so it maybe can not show line in a console line.
    */
    void show_matrix(int mode = 0);

    /*
    show matrix with some wall.
    - it print the result in console,so it maybe can not show line in a console line.
    - the data maybe other values
    */
    void show_matrix_wall();

    void rand_main_path2();
private:
    // the data without walls
    int** matrix;
    // the data with walls
    int** matrix_wall;
    // m_x_max is the width, m_y_max is height 
    int m_x_max = 0, m_y_max = 0;
    std::random_device rd;
    std::mt19937 gen;
    struct PathPoint start_point, end_point, real_end_point;
    long steps_r_min = 0, steps_r_max = 0;
    vector<struct PathPoint> path_main;
    vector<struct PathPoint> path_main_wall;
    vector<struct PathPoint> path_all_base;
    vector<struct PathPoint> path_all_wall;
    vector<struct PathPoint> path_all_wall_expand;
    struct BigArea big_area;

    int rect_width = 7;
    int dead_path_steps = 5;
    int dead_path_water = 95;

    /*
    reset the matrix withou walls
    */
    void reset_matrix_base();
    /*
    rand,get next point
    */
    struct PathPoint next_point(struct PathPoint point, int mode = 0);
    /*
    check current point's avaliable next point.
    - point : current point
    - mode: 0, default, check if the next point can access end
            1, no check
    */
    struct DirectAvaliable check_point_avaliable_direction(struct PathPoint point, int mode = 0);
    /*
    link two point
    */
    void link_2_points_main_path(struct PathPoint point1, struct PathPoint point2);
    /*
    get the rate of paht in matrix withou walls
    */
    int get_path_rate_of_matrix();
    /*
    get the counts of walls in matrix withou walls
    */
    int get_no_zero_point_counts();
    /*
    generate child path start as point, 
    */
    void gen_dead_path_as_point(struct PathPoint point, int steps);
    void rand_dead_path_from_zero_point(int steps);
    void expand_path_main();
    void expand_path_list(vector<struct PathPoint> path_list, int number = 2);
    void add_list_in_path(vector<struct PathPoint> path_list);
    void expand_path_list_to_wall();
    void trans_path_base_to_matrix_wall();

};