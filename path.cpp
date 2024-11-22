#include "path.h"

Path::Path(int x_max, int y_max)
{
    int i = 0, j = 0;
    matrix = (int**)malloc(sizeof(int*) * y_max);
    for (i = 0;i < y_max;i++)
    {
        matrix[i] = (int*)malloc(sizeof(int) * x_max);
    }
    for (i = 0;i < y_max;i++)
    {
        for (j = 0;j < x_max;j++)
        {
            if (i == 0 || i == y_max - 1 || j == 0 || j == x_max - 1)
            {
                matrix[i][j] = 1;
            }
            else
            {
                matrix[i][j] = 0;
            }

        }
    }
    m_x_max = x_max;
    m_y_max = y_max;
    steps_r_min = 2 * (m_x_max + m_y_max);
    steps_r_max = m_x_max * m_y_max / 4;
    gen.seed(rd());
    cout << "initial matrix: width= " << m_x_max << " height= " << m_y_max << endl;
}

Path::~Path()
{

}

void Path::show_matrix(int mode)
{
    int i = 0, j = 0;
    int index = 0;
    for (i = 0;i < m_y_max;i++)
    {
        for (j = 0;j < m_x_max;j++)
        {
            if (matrix[i][j] == 0)
            {
                cout << " " << " ";
            }
            else
            {
                index++;
                cout << " " << matrix[i][j];
            }

        }
        cout << endl;
    }
    cout << endl;
}

void Path::show_matrix_wall()
{
    int i = 0, j = 0;
    int index = 0;
    for (i = 0;i < (m_y_max * 2 - 1);i++)
    {
        for (j = 0;j < (m_x_max * 2 - 1);j++)
        {
            if (matrix_wall[i][j] == 0)
            {
                cout << " " << "";
            }
            else
            {
                index++;
                cout << "" << matrix_wall[i][j];
            }
        }
        cout << endl;
    }
    cout << endl;
}


void Path::set_start_end_point(int start_x, int start_y, int end_x, int end_y)
{
    start_point.x = start_x;
    start_point.y = start_y;
    real_end_point.x = end_x;
    real_end_point.y = end_y;

    matrix[start_point.y][start_point.x] = 1;
    matrix[real_end_point.y][real_end_point.x] = 0;

    if (end_x == 0 || end_x == m_x_max - 1)
    {
        end_point.x = abs(real_end_point.x - 1);
        end_point.y = real_end_point.y;
    }
    if (end_y == 0 || end_y == m_y_max - 1)
    {
        end_point.x = real_end_point.x;
        end_point.y = abs(real_end_point.y - 1);
    }
    cout << "set start and end  point: [ " << start_point.x << " , " << start_point.y << " ] --[ " << real_end_point.x << " , " << real_end_point.y << " ]" << endl;
    cout << "set start and virt point: [ " << start_point.x << " , " << start_point.y << " ] --[ " << end_point.x << " , " << end_point.y << " ]" << endl;
}

void Path::set_steps_range(long step_min, long step_max)
{
    steps_r_min = step_min;
    steps_r_max = step_max;
    cout << "set steps range : " << steps_r_min << " - " << steps_r_max << endl;
}


void Path::rand_main_path()
{
    int i_i = 1;
    struct PathPoint step_point, cur_point;

    bool nearby_end = false;
    while (1)
    {
        gen.seed(rd());
        reset_matrix_base();
        path_main.clear();

        cur_point = start_point;
        path_main.push_back(cur_point);

        nearby_end = false;
        while (1)
        {
            step_point = next_point(cur_point);
            if (step_point == cur_point)
            {
                nearby_end = false;
                //cout << "this path dead. steps= " << path_main.size()<<endl;
                break;
            }

            path_main.push_back(step_point);
            matrix[step_point.y][step_point.x] = 1;

            cur_point = step_point;

            // check dis with end
            if (abs(cur_point.x - end_point.x) + abs(cur_point.y - end_point.y) <= 3)
            {
                if (path_main.size() >= steps_r_min && path_main.size() <= steps_r_max)
                {
                    nearby_end = true;
                    link_2_points_main_path(cur_point, end_point);
                    break;
                }
                if (cur_point == end_point)
                {
                    nearby_end = true;
                    break;
                }
            }
        }
        if (nearby_end)
        {
            if (path_main.size() >= steps_r_min && path_main.size() <= steps_r_max)
            {
                cout << "generate path, steps= " << path_main.size() << endl;
                break;
            }
            else
            {
                cout << "generate path not match, steps= " << path_main.size() << "range: " << steps_r_min << " - " << steps_r_max << endl;
                continue;
            }
        }

    }
    add_list_in_path(path_main);
    show_matrix();
}

void Path::rand_dead_path()
{
    while (1)
    {
        if (get_path_rate_of_matrix() >= dead_path_water)
        {
            break;
        }
        rand_dead_path_from_zero_point(dead_path_steps);
    }
    cout << "gen dead path fill end" << endl;
    trans_path_base_to_matrix_wall();
}


/*
private
*/

void Path::reset_matrix_base()
{
    int i = 0, j = 0;
    for (i = 0;i < m_y_max;i++)
    {
        for (j = 0;j < m_x_max;j++)
        {
            if (i == 0 || i == m_y_max - 1 || j == 0 || j == m_x_max - 1)
            {
                matrix[i][j] = 1;
            }
            else
            {
                matrix[i][j] = 0;
            }
        }
    }
    matrix[start_point.y][start_point.x] = 1;
    matrix[real_end_point.y][real_end_point.x] = 0;
}

struct PathPoint Path::next_point(struct PathPoint point, int mode)
{
    struct DirectAvaliable temp;
    struct PathPoint next_p;
    temp = check_point_avaliable_direction(point, mode);
    int i = 0;
    int dir = 0;
    int temp_c = 0;
    std::uniform_int_distribution<> dis(1, 12);
    if (temp.dir_counts > 0)
    {
        dir = dis(gen) % temp.dir_counts + 1;
        temp_c = 0;
        if (temp.x_l == 1)
        {
            temp_c++;
            if (temp_c == dir)
            {
                next_p.x = point.x - 1;
                next_p.y = point.y;
                return next_p;
            }
        }
        if (temp.x_h == 1)
        {
            temp_c++;
            if (temp_c == dir)
            {
                next_p.x = point.x + 1;
                next_p.y = point.y;
                return next_p;
            }
        }
        if (temp.y_l == 1)
        {
            temp_c++;
            if (temp_c == dir)
            {
                next_p.x = point.x;
                next_p.y = point.y - 1;
                return next_p;
            }
        }
        if (temp.y_h == 1)
        {
            temp_c++;
            if (temp_c == dir)
            {
                next_p.x = point.x;
                next_p.y = point.y + 1;
                return next_p;
            }
        }
        return point;
    }
    else
    {
        return point;
    }
}

struct DirectAvaliable Path::check_point_avaliable_direction(struct PathPoint point, int mode)
{
    struct DirectAvaliable point_dir;
    // x 
    if (point.x == 0)
    {
        point_dir.x_l = 0;
        point_dir.dir_counts--;
        if (matrix[point.y][point.x + 1] == 1)
        {
            point_dir.x_h = 0;
            point_dir.dir_counts--;
        }
        else
        {
            point_dir.x_h = 1;
        }
    }
    if (point.x == m_x_max - 1)
    {
        point_dir.x_h = 0;
        point_dir.dir_counts--;
        if (matrix[point.y][point.x - 1] == 1)
        {
            point_dir.x_l = 0;
            point_dir.dir_counts--;
        }
        else
        {
            point_dir.x_l = 1;
        }
    }
    if (point.x != 0 && point.x != m_x_max - 1)
    {
        if (matrix[point.y][point.x - 1] == 1)
        {
            point_dir.x_l = 0;
            point_dir.dir_counts--;
        }
        else
        {
            point_dir.x_l = 1;
        }
        if (matrix[point.y][point.x + 1] == 1)
        {
            point_dir.x_h = 0;
            point_dir.dir_counts--;
        }
        else
        {
            point_dir.x_h = 1;
        }
    }

    // y
    if (point.y == 0)
    {
        point_dir.y_l = 0;
        point_dir.dir_counts--;
        if (matrix[point.y + 1][point.x] == 1)
        {
            point_dir.y_h = 0;
            point_dir.dir_counts--;
        }
        else
        {
            point_dir.y_h = 1;
        }
    }
    if (point.y == m_y_max - 1)
    {
        point_dir.y_h = 0;
        point_dir.dir_counts--;
        if (matrix[point.y - 1][point.x] == 1)
        {
            point_dir.y_l = 0;
            point_dir.dir_counts--;
        }
        else
        {
            point_dir.y_l = 1;
        }
    }
    if (point.y != 0 && point.y != m_y_max - 1)
    {
        if (matrix[point.y - 1][point.x] == 1)
        {
            point_dir.y_l = 0;
            point_dir.dir_counts--;
        }
        else
        {
            point_dir.y_l = 1;
        }
        if (matrix[point.y + 1][point.x] == 1)
        {
            point_dir.y_h = 0;
            point_dir.dir_counts--;
        }
        else
        {
            point_dir.y_h = 1;
        }
    }
    if (mode == 1)
    {
        return point_dir;
    }

    // check dead path
    int i = 0;
    if (point_dir.dir_counts == 2)
    {
        if (point_dir.x_l == 1)
        {
            for (i = 1;i < point.x - 1;i++)
            {
                if (matrix[point.y][i] == 1)
                {
                    point_dir.x_l = 2;
                    point_dir.dir_counts--;
                    break;
                }
            }
        }
        if (point_dir.x_h == 1)
        {
            for (i = point.x + 1;i < m_x_max - 1;i++)
            {
                if (matrix[point.y][i] == 1)
                {
                    point_dir.x_h = 2;
                    point_dir.dir_counts--;
                    break;
                }
            }
        }
        if (point_dir.y_l == 1)
        {
            for (i = 1;i < point.y - 1;i++)
            {
                if (matrix[i][point.x] == 1)
                {
                    point_dir.y_l = 2;
                    point_dir.dir_counts--;
                    break;
                }
            }
        }
        if (point_dir.y_h == 1)
        {
            for (i = point.y + 1;i < m_y_max - 1;i++)
            {
                if (matrix[i][point.x] == 1)
                {
                    point_dir.y_h = 2;
                    point_dir.dir_counts--;
                    break;
                }
            }
        }
    }
    return point_dir;
}

void Path::link_2_points_main_path(struct PathPoint point1, struct PathPoint point2)
{
    int min_x = point1.x < point2.x ? point1.x : point2.x;
    int max_x = point1.x > point2.x ? point1.x : point2.x;
    int min_y = point1.y < point2.y ? point1.y : point2.y;
    int max_y = point1.y > point2.y ? point1.y : point2.y;
    int i = 0;
    //int y = 0;
    struct PathPoint temp;
    for (i = point1.x;i != point2.x;i = i + (point2.x - point1.x) / abs(point2.x - point1.x))
    {
        temp.y = point1.y;
        temp.x = i;
        matrix[temp.y][temp.x] = 1;
        path_main.push_back(temp);
    }
    for (i = point1.y;i != point2.y;i = i + (point2.y - point1.y) / abs(point2.y - point1.y))
    {
        temp.y = i;
        temp.x = point2.x;
        matrix[temp.y][temp.x] = 1;
        path_main.push_back(temp);
    }
    path_main.push_back(end_point);
    path_main.push_back(real_end_point);
}

int Path::get_path_rate_of_matrix()
{
    int i = 0, j = 0;
    long sum = 0;
    for (i = 0;i < m_y_max;i++)
    {
        for (j = 0;j < m_x_max;j++)
        {
            if (matrix[i][j] != 0)
            {
                sum++;
            }
        }
    }
    return sum * 100 / m_y_max / m_x_max;
}

void Path::rand_dead_path_from_zero_point(int steps)
{
    vector<struct PathPoint> point_list, path_list;

    struct PathPoint temp;
    int i = 0, j = 0;
    bool is_closed_exist = false;
    int dir = 0;
    for (i = 1;i < m_y_max - 1;i++)
    {
        for (j = 1;j < m_x_max - 1;j++)
        {
            if (matrix[i][j] == 0)
            {
                is_closed_exist = false;
                if (matrix[i - 1][j] == 1 && i != 1)
                {
                    is_closed_exist = true;
                }
                else
                {
                    if (matrix[i + 1][j] == 1 && i != m_y_max - 2)
                    {
                        is_closed_exist = true;
                    }
                    else
                    {
                        if (matrix[i][j - 1] == 1 && j != 1)
                        {
                            is_closed_exist = true;
                        }
                        else
                        {
                            if (matrix[i][j + 1] == 1 && j != m_x_max - 2)
                            {
                                is_closed_exist = true;
                            }
                        }
                    }
                }
                if (is_closed_exist)
                {
                    temp.x = j;temp.y = i;
                    point_list.push_back(temp);
                }
            }
        }
    }

    std::uniform_int_distribution<> dis(0, (int)(point_list.size() - 1));
    temp = point_list[dis(gen)];

    struct PathPoint step_point, cur_point = temp;

    if (matrix[temp.y - 1][temp.x] == 1 && temp.y != 1)
    {
        cur_point.y--;
    }
    else
    {
        if (matrix[temp.y + 1][temp.x] == 1 && temp.y != m_y_max - 2)
        {
            cur_point.y++;
        }
        else
        {
            if (matrix[temp.y][temp.x - 1] == 1 && temp.x != 1)
            {
                cur_point.x--;
            }
            else
            {
                if (matrix[temp.y][temp.x + 1] == 1 && temp.x != m_x_max - 2)
                {
                    cur_point.x++;
                }
            }
        }
    }

    path_list.push_back(cur_point);
    path_list.push_back(temp);
    cur_point = temp;
    matrix[temp.y][temp.x] = 1;
    for (i = 0; i < steps; i++)
    {
        step_point = next_point(cur_point, 1);
        if (step_point == cur_point)
        {
            break;
        }
        matrix[step_point.y][step_point.x] = 1;
        cur_point = step_point;
        path_list.push_back(cur_point);
    }
    add_list_in_path(path_list);
}

void Path::gen_dead_path_as_point(struct PathPoint point, int steps)
{
    vector<struct PathPoint> path_list;
    struct PathPoint step_point, cur_point;
    int i = 0;
    cur_point = point;
    path_list.push_back(cur_point);

    for (i = 0; i < steps; i++)
    {
        step_point = next_point(cur_point, 1);
        if (step_point == cur_point)
        {
            break;
        }
        matrix[step_point.y][step_point.x] = 1;
        cur_point = step_point;
        path_list.push_back(cur_point);
    }
    add_list_in_path(path_list);
}


void Path::add_list_in_path(vector<struct PathPoint> path_list)
{
    struct PathPoint zeroPoint;
    zeroPoint.x = -1;zeroPoint.y = -1;
    path_all_base.push_back(zeroPoint);
    int i = 0;
    for (i = 0;i < path_list.size();i++)
    {
        path_all_base.push_back(path_list[i]);
    }
}

void Path::expand_path_list_to_wall()
{
    struct PathPoint temp;
    int i = 0;
    for (i = 0;i < path_all_base.size();i++)
    {
        temp.x = path_all_base[i].x * 2;
        temp.y = path_all_base[i].y * 2;
        path_all_wall.push_back(temp);
    }

}

void Path::trans_path_base_to_matrix_wall()
{
    expand_path_list_to_wall();
    int i = 0;
    bool is_start = true;
    bool is_main_path = true;
    struct PathPoint temp;
    for (i = 0;i < path_all_wall.size();i++)
    {
        if (path_all_wall[i].x >= 0 && path_all_wall[i].y >= 0)
        {
            if (is_start)
            {
                path_all_wall_expand.push_back(path_all_wall[i]);
                is_start = false;
            }
            else
            {
                temp.x = (path_all_wall[i].x + path_all_wall[i - 1].x) / 2;
                temp.y = (path_all_wall[i].y + path_all_wall[i - 1].y) / 2;
                path_all_wall_expand.push_back(temp);
                if (is_main_path)
                {
                    matrix_wall[(path_all_wall[i].y + path_all_wall[i - 1].y) / 2][(path_all_wall[i].x + path_all_wall[i - 1].x) / 2] = 1;
                }
                else
                {
                    matrix_wall[(path_all_wall[i].y + path_all_wall[i - 1].y) / 2][(path_all_wall[i].x + path_all_wall[i - 1].x) / 2] = 3;
                }

                path_all_wall_expand.push_back(path_all_wall[i]);
            }
        }
        else
        {
            is_start = true;
            is_main_path = false;
            path_all_wall_expand.push_back(path_all_wall[i]);
        }
    }
}


void Path::gen_matrix_wall_main_path()
{
    cout << " gen matrix wall" << endl;
    int i = 0, j = 0;
    matrix_wall = (int**)malloc(sizeof(int*) * (m_y_max * 2 - 1));
    for (i = 0;i < (m_y_max * 2 - 1);i++)
    {
        matrix_wall[i] = (int*)malloc(sizeof(int) * (m_x_max * 2 - 1));
    }
    cout << " initial matrix wall" << endl;
    for (i = 0;i < (m_y_max * 2 - 1);i++)
    {
        for (j = 0;j < (m_x_max * 2 - 1);j++)
        {
            if (i == 0 || i == 2 * m_y_max - 2 || j == 0 || j == 2 * m_x_max - 2)
            {
                matrix_wall[i][j] = 0;
            }
            else
            {
                matrix_wall[i][j] = 0;
            }
        }
    }
    for (i = 0;i < (m_y_max * 2 - 1);i = i + 2)
    {
        for (j = 0;j < (m_x_max * 2 - 1);j = j + 2)
        {
            if (i == 0 || i == 2 * m_y_max - 2 || j == 0 || j == 2 * m_x_max - 2)
            {
                matrix_wall[i][j] = 0;
            }
            else {
                matrix_wall[i][j] = matrix[i / 2][j / 2];
            }
        }
    }
    cout << " initial end matrix wall" << endl;
    expand_path_main();
    cout << " expand end matrix wall" << endl;
}

void Path::expand_path_main()
{
    int i = 1;
    struct PathPoint temp, temp_pre, temp_mid;
    temp.x = path_main[0].x * 2;
    temp.y = path_main[0].y * 2;
    temp_pre.x = path_main[0].x * 2;
    temp_pre.y = path_main[0].y * 2;
    matrix_wall[temp.y][temp.x] = 1;
    path_main_wall.push_back(temp);

    for (i = 1;i < path_main.size();i++)
    {
        temp.x = path_main[i].x * 2;
        temp.y = path_main[i].y * 2;

        temp_mid.x = (temp.x + temp_pre.x) / 2;
        temp_mid.y = (temp.y + temp_pre.y) / 2;

        matrix_wall[temp_mid.y][temp_mid.x] = 1;
        matrix_wall[temp.y][temp.x] = 1;
        path_main_wall.push_back(temp_mid);
        path_main_wall.push_back(temp);

        temp_pre.x = temp.x;
        temp_pre.y = temp.y;
    }

}


void Path::expand_path_list(vector<struct PathPoint> path_list, int number)
{
    int i = 1;
    struct PathPoint temp, temp_pre, temp_mid;
    temp.x = path_list[0].x * 2;
    temp.y = path_list[0].y * 2;
    temp_pre.x = path_list[0].x * 2;
    temp_pre.y = path_list[0].y * 2;
    matrix_wall[temp.y][temp.x] = 1;

    for (i = 1;i < path_list.size();i++)
    {
        temp.x = path_list[i].x * 2;
        temp.y = path_list[i].y * 2;

        temp_mid.x = (temp.x + temp_pre.x) / 2;
        temp_mid.y = (temp.y + temp_pre.y) / 2;

        matrix_wall[temp_mid.y][temp_mid.x] = number;
        matrix_wall[temp.y][temp.x] = number;


        temp_pre.x = temp.x;
        temp_pre.y = temp.y;
    }
}

void Path::show_gui_image(bool main_path_enable, bool other_path_enable, string fname)
{

    cv::Mat image = cv::Mat::zeros((m_y_max * 2 - 1) * rect_width, (m_x_max * 2 - 1) * rect_width, CV_8UC3);
    image.setTo(cv::Scalar(255, 255, 255));

    cv::Mat rect = cv::Mat::zeros(rect_width, rect_width, CV_8UC3);
    rect.setTo(cv::Scalar(255, 255, 255));

    cv::Scalar lineColorW(0, 0, 0);
    cv::Scalar emptyColor(255, 255, 255);
    cv::Scalar lineColorR(0, 0, 255);
    cv::Scalar lineColorD(255, 255, 0);

    int thickness = 1;

    int i = 0, j = 0;
    for (i = 1;i < (m_y_max * 2 - 2);i++)
    {
        for (j = 1;j < (m_x_max * 2 - 2);j++)
        {
            if (matrix_wall[i][j] == 0)
            {
                // up down
                cv::line(rect, cv::Point(rect_width / 2, 0), cv::Point(rect_width / 2, rect_width), emptyColor, thickness);
                // left right
                cv::line(rect, cv::Point(0, rect_width / 2), cv::Point(rect_width, rect_width / 2), emptyColor, thickness);
                // left
                if (j > 0)
                {
                    if (matrix_wall[i][j - 1] == 0)
                    {
                        cv::line(rect, cv::Point(0, rect_width / 2), cv::Point(rect_width / 2, rect_width / 2), lineColorW, thickness);
                    }
                }
                // right
                if (j < (m_x_max * 2 - 2))
                {
                    if (matrix_wall[i][j + 1] == 0)
                    {
                        cv::line(rect, cv::Point(rect_width / 2, rect_width / 2), cv::Point(rect_width, rect_width / 2), lineColorW, thickness);
                    }
                }
                // top
                if (i > 0)
                {
                    if (matrix_wall[i - 1][j] == 0)
                    {
                        cv::line(rect, cv::Point(rect_width / 2, 0), cv::Point(rect_width / 2, rect_width / 2), lineColorW, thickness);
                    }
                }
                // bot
                if (i < m_y_max * 2 - 2)
                {
                    if (matrix_wall[i + 1][j] == 0)
                    {
                        cv::line(rect, cv::Point(rect_width / 2, rect_width), cv::Point(rect_width / 2, rect_width / 2), lineColorW, thickness);
                    }
                }
                cv::Rect roi((j * rect_width), (i * rect_width), rect_width, rect_width);
                rect.copyTo(image(roi));
            }
        }
    }
    string filename = fname + "_q.png";
    cv::imwrite(filename, image);
    if (other_path_enable)
    {
        for (i = 1;i < (m_y_max * 2 - 2);i++)
        {
            for (j = 1;j < (m_x_max * 2 - 2);j++)
            {
                if (matrix_wall[i][j] == 3)
                {
                    // up down
                    cv::line(rect, cv::Point(rect_width / 2, 0), cv::Point(rect_width / 2, rect_width), emptyColor, thickness);
                    // left right
                    cv::line(rect, cv::Point(0, rect_width / 2), cv::Point(rect_width, rect_width / 2), emptyColor, thickness);

                    // left
                    if (j > 0)
                    {
                        if (matrix_wall[i][j - 1] == 3)
                        {
                            cv::line(rect, cv::Point(0, rect_width / 2), cv::Point(rect_width / 2, rect_width / 2), lineColorD, thickness);
                        }
                    }
                    // right
                    if (j < (m_x_max * 2 - 2))
                    {
                        if (matrix_wall[i][j + 1] == 3)
                        {
                            cv::line(rect, cv::Point(rect_width / 2, rect_width / 2), cv::Point(rect_width, rect_width / 2), lineColorD, thickness);
                        }
                    }
                    // top
                    if (i > 0)
                    {
                        if (matrix_wall[i - 1][j] == 3)
                        {
                            cv::line(rect, cv::Point(rect_width / 2, 0), cv::Point(rect_width / 2, rect_width / 2), lineColorD, thickness);
                        }
                    }
                    // bot
                    if (i < m_y_max * 2 - 2)
                    {
                        if (matrix_wall[i + 1][j] == 3)
                        {
                            cv::line(rect, cv::Point(rect_width / 2, rect_width), cv::Point(rect_width / 2, rect_width / 2), lineColorD, thickness);
                        }
                    }
                    cv::Rect roi((j * rect_width), (i * rect_width), rect_width, rect_width);
                    rect.copyTo(image(roi));
                }
            }
        }
    }
    if (main_path_enable)
    {
        cv::Point main_p1, main_p2;
        main_p1.x = path_main_wall[0].x * rect_width;
        main_p1.y = path_main_wall[0].y * rect_width;
        for (i = 1;i < path_main_wall.size();i++)
        {
            main_p2.x = path_main_wall[i].x * rect_width;
            main_p2.y = path_main_wall[i].y * rect_width;
            cv::line(image, main_p1, main_p2, lineColorR, thickness);
            main_p1 = main_p2;
        }
    }

    cv::imshow("Composite Image", image);
    // 等待按键按下
    cv::waitKey(0);


    filename = fname + "_a.png";
    cv::imwrite(filename, image);

}

int Path::get_no_zero_point_counts()
{
    int i = 0, j = 0;
    int sum = 0;
    for (i = 0;i < m_y_max;i++)
    {
        for (j = 0;j < m_x_max;j++)
        {
            if (matrix[i][j] == 1)
            {
                sum++;
            }

        }
    }
    return sum;
}

void Path::matrix_wall_edge_zero()
{
    int i = 0;
    for (i = 0;i < (m_x_max * 2 - 1);i++)
    {
        matrix_wall[0][i] = 1;
        matrix_wall[m_y_max * 2 - 2][i] = 1;
    }
    for (i = 0;i < (m_y_max * 2 - 1);i++)
    {
        matrix_wall[i][0] = 1;
        matrix_wall[i][m_x_max * 2 - 2] = 1;
    }
}

void Path::matrix_wall_merge_area()
{
    int i = 0, j = 0;
    for (i = 0;i < m_y_max - 1;i++)
    {
        for (j = 0;j < m_x_max - 1;j++)
        {
            if (matrix[i][j] == 0)
            {
                matrix_wall[i * 2][j * 2] = 4;
                if (matrix[i + 1][j] == 0)
                {
                    matrix_wall[i * 2 + 1][j * 2] = 4;
                }
                if (matrix[i][j + 1] == 0)
                {
                    matrix_wall[i * 2][j * 2 + 1] = 4;
                }
            }
        }
    }
}

void Path::matrix_wall_re()
{
    int i = 0, j = 0;
    for (i = 0;i < (m_y_max * 2 - 1);i++)
    {
        for (j = 0;j < (m_x_max * 2 - 1); j++)
        {
            matrix_wall[i][j] = 0;
        }
    }
    bool is_start = true;
    bool is_main_path = true;
    //struct PathPoint temp;
    for (i = 0;i < path_all_wall_expand.size();i++)
    {
        if (path_all_wall_expand[i].x >= 0 && path_all_wall_expand[i].y >= 0)
        {
            if (is_start)
            {
                is_start = false;
            }
            else
            {
                if (is_main_path)
                {
                    matrix_wall[path_all_wall_expand[i].y][path_all_wall_expand[i].x] = 1;
                }
                else
                {
                    matrix_wall[path_all_wall_expand[i].y][path_all_wall_expand[i].x] = 3;
                }
            }
        }
        else
        {
            is_start = true;
            is_main_path = false;

        }
    }
}

void Path::set_show_gui(int width, int height)
{
    //int min_len = width > height ? height : width;
    int rect_x = width / m_x_max / 2;
    int rect_y = height / m_y_max / 2;

    int min_w = rect_x < rect_y ? rect_x : rect_y;
    min_w = (min_w / 2 * 2) + 1;
    if (min_w <= 5)
    {
        rect_width = 5;
    }
    else
    {
        rect_width = min_w;
    }
}

void Path::set_deep_and_water(int deep, int water)
{
    dead_path_steps = deep;
    dead_path_water = water;
}

void Path::rand_main_path2()
{
    vector<struct PathPoint> path_list;

}