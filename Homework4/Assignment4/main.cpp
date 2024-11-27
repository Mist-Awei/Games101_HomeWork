#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 5) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm
    if (control_points.size() == 2)
    {
        //�������Ƶ�ʱ����t�����ĵ�p_t
        auto p_t = control_points[0] + t * (control_points[1] - control_points[0]);
        return p_t;
    }

    std::vector<cv::Point2f> t_points;
    //ʣ���������Ƶ�����������µĿ��Ƶ���еݹ�
    for (size_t i = 0; i < control_points.size() - 1; i++)
    {
        t_points.push_back(control_points[i] + t * (control_points[i + 1] - control_points[i]));
    }

    return recursive_bezier(t_points, t);
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    for (double t = 0.0; t <= 1.0; t += 0.001)
    {
        auto point = recursive_bezier(control_points, t);

        window.at<cv::Vec3b>(point.y, point.x)[1] = 255;
    }
}

void bezier_aa(const std::vector<cv::Point2f>& control_points, cv::Mat& window)
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    for (double t = 0.0; t <= 1.0; t += 0.001)
    {
        auto point = recursive_bezier(control_points, t);

        window.at<cv::Vec3b>(point.y, point.x)[1] = 255;
        //��¼���ص�����
        int x_pos = point.x, y_pos = point.y;
        //������Χ�Ÿ����ؾ�point�������صľ����ֵ��Ⱦ��ɫ��ͬ��������ɫȡ���ֵ
        for (int dx = -1; dx <= 1; dx++)
        {
            for (int dy = -1; dy <= 1; dy++)
            {
                //����Խ��
                if (x_pos + dx < 0 || x_pos + dx>700 || y_pos + dy < 0 || y_pos + dy>700) continue;

                double dx_pos = point.x - (x_pos + dx + 0.5), dy_pos = point.y - (y_pos + dy + 0.5);
                double d = std::sqrt(dx_pos * dx_pos + dy_pos * dy_pos);
                double ratio = 1 - std::sqrt(2) * d / 3.0;
                window.at<cv::Vec3b>(point.y + dy, point.x + dx)[1] = std::fmax(window.at<cv::Vec3b>(point.y + dy, point.x + dx)[1], 255 * ratio);
            }
        }
    }

}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 5) 
        {
            //ģ��
            //naive_bezier(control_points, window);
            //����������
            //bezier(control_points, window);
            //�����������ı���������
            bezier_aa(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
