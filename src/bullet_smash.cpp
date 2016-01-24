/**

  g++ bullet_smash.cpp -lopencv_imgproc -lopencv_highgui -lopencv_core && ./a.out
*/

#include <keyboard/Key.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>

class Player
{
  cv::Point pos_;
  cv::Point vel_;
  cv::Size size_;
  bool on_ground_;

  ros::NodeHandle nh_;

  ros::Subscriber key_sub_;
  void keyCallback(const keyboard::Key::ConstPtr& msg);

public:
  Player(const cv::Point start_pos);

  void update();
  void move(cv::Point dxy);
  void accel(cv::Point dxy);
  void walk(const float vel);
  void jump();
  void draw(cv::Mat& screen);
};

Player::Player(const cv::Point start_pos) :
    pos_(start_pos),
    vel_(0, 0),
    size_(16, 16),
    on_ground_(false)
{
  key_sub_ = nh_.subscribe<keyboard::Key>("/keyboard/keydown", 1,
      &Player::keyCallback, this);
}

void Player::keyCallback(const keyboard::Key::ConstPtr& msg)
{
  std::cout << "test" << msg->code << std::endl;
  ROS_INFO_STREAM(msg->code);
}

void Player::update()
{
  cv::Point gravity = cv::Point(0, -1);
  vel_ += gravity;
  pos_ += vel_;

  const int ground = 16;
  if (pos_.y < ground)
  {
    on_ground_ = true;
    pos_.y = ground;
    vel_.y = 0;
  }
  else
  {
    on_ground_ = false;
  }

  if (pos_.x < 0)
  {
    vel_.x = 0;
    pos_.x = 0;
  }
  const int width = 700;
  if (pos_.x > width)
  {
    vel_.x = 0;
    pos_.x = width;
  }

  // air resistance
  vel_ *= 0.95;

  // ground friction
  if (on_ground_)
    vel_.x *= 0.8;
}

void Player::draw(cv::Mat& screen)
{
  cv::Point screen_pos = pos_;
  screen_pos.y = screen.rows - pos_.y;
  cv::rectangle(screen, screen_pos, screen_pos + cv::Point(size_),
      cv::Scalar(128, 128, 128), CV_FILLED);
}

void Player::accel(cv::Point dxy)
{
  vel_ += dxy;
}

void Player::walk(const float vel)
{
  accel(cv::Point(vel, 0));
  // limit mid air acceleration 
  if (!on_ground_)
    vel_.x *= 0.8;
}

void Player::jump()
{
  if (on_ground_ == true)
  {
    accel(cv::Point(0, 15));
  }
}

void Player::move(cv::Point dxy)
{
  pos_ += dxy;
}

void keyCallback(const keyboard::Key::ConstPtr& msg)
{
  ROS_INFO_STREAM(msg->code);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bullet_smash");

  cv::Mat background(cv::Size(1000, 700), CV_8UC3, cv::Scalar(255, 255, 105));

  Player player(cv::Point(16, 16));

  ros::spin();

  ros::Rate rate(10);

  while (ros::ok())
  {
    #if 0
    cv::Mat screen = background.clone();
    player.update();
    player.draw(screen);

    cv::imshow("bullet_smash", screen);

    // int key = 0;
    int key = cv::waitKey(5);
    if (key > 0)
      ROS_INFO_STREAM(char(key));
    if (key == 'a')
    {
      player.walk(-3);
    }
    if (key == 'd')
    {
      player.walk(3);
    }
    if (key == 'w')
    {
      player.jump();
    }
    #endif

    // ros::Duration(0.5).sleep();
    // rate.sleep();
  }
}
