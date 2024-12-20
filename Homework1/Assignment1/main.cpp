#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    //Eigen::Matrix4f rotation;
    
    float a = rotation_angle / 180.0 * MY_PI;
    model << cos(a), -sin(a), 0, 0,
        sin(a), cos(a), 0, 0, 
        0, 0, 1, 0,
        0, 0, 0, 1;
    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.

    float n = -zNear;
    float f = -zFar;
    Eigen::Matrix4f Persp_Ortho;
    Persp_Ortho << n, 0, 0, 0,
        0, n, 0, 0,
        0, 0, n + f, -n * f,
        0, 0, 1, 0;
    float l, r, t, b;
    t = tan(eye_fov * 0.5 / 180 * MY_PI) * abs(n);
    b = -t;
    r = t * aspect_ratio;
    l = -r;

    Eigen::Matrix4f Otrans,Oscale;
    Otrans << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, -(n + f) / 2,
        0, 0, 0, 1;
    Oscale << 2 / (r - l), 0, 0, 0,
        0, 2 / (t - b), 0, 0,
        0, 0, 2 / (n - f), 0,
        0, 0, 0, 1;
    projection = Oscale * Otrans * Persp_Ortho * projection;

    return projection;
}

Eigen::Matrix4f get_rotation(Vector3f axis, float angle)
{
    float a = angle / 180 * MY_PI;
    Eigen::Matrix4f I, N, Rota;
    Eigen::Vector4f axis4;
    axis4 << axis.x(), axis.y(), axis.z(), 0;
    I = Eigen::Matrix4f::Identity();
    N << 0, -axis.z(), axis.y(), 0,
        axis.z(), 0, -axis.x(), 0,
        -axis.y(), axis.x(), 0, 0,
        0, 0, 0, 1;
    Rota = cos(a) * I + (1 - cos(a)) * axis4 * axis4.transpose() + sin(a) * N;
    Rota(3, 3) = 1;
    return Rota;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    float rangle = 0;
    bool command_line = false;
    bool rflag = false;
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

    std::vector<Eigen::Vector3f> pos{{-2, 0, -2}, {0, 2, -2}, {2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    Eigen::Vector3f raxis;

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

        if (rflag) {
            r.set_rodrigues(get_rotation(raxis, rangle));
        }
        else {
            r.set_rodrigues(get_rotation({ 0,0,1 }, 0));
        }

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';
        
        if (key == 'a') {
            if (rflag) {
                rangle += 10;
            }
            else angle += 10;
        }
        else if (key == 'd') {
            if (rflag) {
                rangle -= 10;
            }
            else angle -= 10;
        }
        else if (key == 'r') {
            rflag = true;
            std::cout << "Please enter the axis:" << std::endl;
            std::cin >> raxis.x() >> raxis.y() >> raxis.z();
        }
    }
    return 0;
}
