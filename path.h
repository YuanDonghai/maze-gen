#pragma once
#include <iostream>
#include <random>
#include <vector>
#include <windows.h>
#include <thread>


#include <opencv2/opencv.hpp>

using namespace std;

#define MULTHREAD 8 // 4*2

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
    int selected = 0;
    DirectAvaliable& operator=(const DirectAvaliable& other)
    {
        if (this != &other)
        {
            x_h = other.x_h;
            x_l = other.x_l;
            y_h = other.y_h;
            y_l = other.y_l;
            dir_counts = other.dir_counts;
            selected = other.selected;
        }
        return *this;
    }
};

struct BigArea {
    struct PathPoint start_point;
    int width;
};

struct RandPointTag {
    int index;
    int left_times = 10;
};

struct PthreadS {
    struct PathPoint point;
    int index = 0;
    int res = 0;
};

class Path {
public:
    Path(int x_max, int y_max);
    ~Path();

    void start(int start_x, int start_y, int end_x, int end_y);

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
    vector<struct DirectAvaliable> path_main_selected;
    vector<struct PathPoint> path_main_wall;
    vector<struct PathPoint> path_all_base;
    vector<struct PathPoint> path_all_wall;
    vector<struct PathPoint> path_all_wall_expand;
    //struct BigArea big_area;

    int rect_width = 7;
    int dead_path_steps = 5;
    int dead_path_water = 95;
    int try_steps = 5;

    vector<struct RandPointTag> path_main_break_points;
    vector<struct PathPoint> path_edge_list_n, path_edge_list_r;

    cv::Mat rect_list[64];
    vector<int> rect_index;

    std::thread dead_path_threads[MULTHREAD];
    int dead_path_threads_res[MULTHREAD];


    cv::Mat save_image[MULTHREAD];
    //long long s_t = GetCurrentTimeInMillis();

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
    void show_gui_image(bool show_image = true, bool main_path_enable = false, bool other_path_enable = false, string fname = "m1");
    void show_gui_image_fast(bool show_image = true, bool main_path_enable = false, bool other_path_enable = false, string fname = "m1");
    void show_gui_image_line(bool show_image = true, bool main_path_enable = false, bool other_path_enable = false, string fname = "m1");
    /*
    reset the matrix withou walls
    */
    void reset_matrix_base();
    /*
    */
    void reset_matrix_with_main_path();

    /*
    rand,get next point
    */
    struct PathPoint next_point(struct PathPoint point, int mode = 0);
    struct PathPoint re_next_point(struct PathPoint point);
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
    //void gen_dead_path_as_point(struct PathPoint point, int steps);
    int rand_dead_path_from_zero_point(int steps);
    int rand_dead_path_from_zero_point_f(int steps);
    int rand_dead_path_from_zero_point_m(int steps);
    void expand_path_main();
    void expand_path_list(vector<struct PathPoint> path_list, int number = 2);
    void add_list_in_path(vector<struct PathPoint> path_list);
    void expand_path_list_to_wall();
    void trans_path_base_to_matrix_wall();
    int get_direct_of_main_list(struct PathPoint point, int x);
    long long GetCurrentTimeInMillis();

    void initial_rect_style();
    int get_rect_index(struct PathPoint point);
    int get_rect_index(int x, int y);

    void dead_thread(void* th_p);
};

