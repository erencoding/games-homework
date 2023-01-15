#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Vector3f  rotation_axis = {1,0,0};

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_rotation(Vector3f axis, float angle) 
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
    float nx = axis[0];
    float ny = axis[1];
    float nz = axis[2];
    Eigen::Matrix<float, 1, 3> n;
    Eigen::Matrix3f N;
    N << 0, -nz, ny,
        nz, 0, -nx,
        -ny, nx, 0;

    float a = angle * MY_PI / 180;
    Eigen::Matrix3f R = cos(a) * I + (1 - cos(a)) * axis * axis.transpose() + sin(a) * N;
     model <<
        R(0, 0), R(0, 1), R(0, 2), 0,
        R(1, 0), R(1, 1), R(1, 2), 0,
        R(2, 0), R(2, 1), R(2, 2), 0,
        0, 0, 0, 1;
     return model;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f transform;
    /** 普通做法：围绕z轴旋转 */
    // rotation_angle = rotation_angle *  MY_PI / 180;
    // transform << cos(rotation_angle), -sin(rotation_angle), 0, 0,
    //     sin(rotation_angle), cos(rotation_angle), 0, 0,
    //     0, 0, 1, 0,
    //     0, 0, 0, 1;
    // model = transform * model;

    /** 提高：围绕任意轴旋转*/
    transform = get_rotation(rotation_axis, rotation_angle);
    std::cout << "transform:" << transform << "\n\n";
    std::cout << "axis:" << rotation_angle << "\n\n";
    model = transform * model;

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f Mortho1 = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f Mortho2 = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f Mortho = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f Mpersp_ortho = Eigen::Matrix4f::Identity();

    //  Calculate t r l b
    float fov = eye_fov * MY_PI / 180;
    float t =  -tan(fov / 2) * abs(zNear);
    float r = aspect_ratio * t;
    float l = -r;
    float b = -t;

    Mortho1 << 2 / (r - l), 0, 0, 0,
                    0, 2 / (t - b), 0, 0,
                        0, 0, 2 / (zNear - zFar), 0,
                        0, 0, 0, 1;
    Mortho2 << 1, 0, 0, -(r + l) / 2,
                        0, 1, 0, -(t + b) / 2,
                        0, 0, 1, -(zNear + zFar) / 2,
                        0, 0, 0, 1;
    Mortho = Mortho1 * Mortho2;

    Mpersp_ortho << zNear, 0, 0, 0,
                                0, zNear, 0, 0,
                                0, 0, zNear + zFar, -zNear * zFar,
                                0, 0, 1, 0;

    projection = Mortho * Mpersp_ortho * projection;

    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);


        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
