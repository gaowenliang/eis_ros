#include "eis.h"

Eis::Eis( string camera_model_file, double pinhole_angle_row, double pinhole_angle_col, int pinhole_max_size )
{
    m_cam = camera_model::CameraFactory::instance( )->generateCameraFromYamlFile( camera_model_file );
    m_eis = new imageCropper( m_cam, pinhole_angle_row, pinhole_angle_col, pinhole_max_size, 1 );

    m_is_eis_on = true;
}

Eis::~Eis( ) { delete m_eis; }

cv::Mat
Eis::process( const cv::Mat image_in, const Eigen::Quaterniond q_Cc )
{
    if ( m_is_eis_on )
        return m_eis->crop( image_in, q_Cc, 0 );
    else
        return image_in;
}

bool
Eis::eisOFF( )
{
    m_is_eis_on = false;
    return true;
}

bool
Eis::eisON( )
{
    m_is_eis_on = true;
    return true;
}
