#ifndef EIS_H
#define EIS_H

#include <camera_model/camera_models/CameraFactory.h>
#include <eigen3/Eigen/Eigen>
#include <image_cropper/imageCropper/imageCropper.h>

#define DEFAULT_ANGLE_ROW 90
#define DEFAULT_ANGLE_COL 60
#define DEFAULT_MAX_SIZE 320

class Eis
{
    public:
    Eis( std::string camera_model_file,
         double pinhole_angle_row = DEFAULT_ANGLE_ROW,
         double pinhole_angle_col = DEFAULT_ANGLE_COL,
         int pinhole_max_size     = DEFAULT_MAX_SIZE );
    ~Eis( );

    cv::Mat process( const cv::Mat image_in, const Eigen::Quaterniond q_Cc );

    bool eisOFF( );
    bool eisON( );

    private:
    bool m_is_eis_on;
    imageCropper* m_eis;
    camera_model::CameraPtr m_cam;
};

#endif // EIS_H
