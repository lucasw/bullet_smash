/**

  g++ bullet_smash.cpp -lopencv_imgproc -lopencv_highgui -lopencv_core && ./a.out
*/

#include <keyboard/Key.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>

class Player
{
  cv::Point2f pos_;
  cv::Point2f vel_;
  cv::Size size_;
  bool on_ground_;

  ros::NodeHandle nh_;

  ros::Subscriber key_up_sub_;
  ros::Subscriber key_down_sub_;
  void keyCallback(const keyboard::Key::ConstPtr& msg, const bool down_not_up);

  bool left_input_;
  bool right_input_;
  bool jump_input_;

public:
  Player(const cv::Point2f start_pos);

  void update(const cv::Mat mask);
  void move(const cv::Point2f dxy);
  void accel(const cv::Point2f dxy);
  void walk(const float vel);
  void jump();
  void draw(cv::Mat& screen);
};

Player::Player(const cv::Point2f start_pos) :
    pos_(start_pos),
    vel_(0.0, 0.0),
    size_(16, 16),
    on_ground_(false),
    left_input_(false),
    right_input_(false),
    jump_input_(false)
{
  key_down_sub_ = nh_.subscribe<keyboard::Key>("keyboard/keydown", 10,
      boost::bind(&Player::keyCallback, this, _1, true));
  key_up_sub_ = nh_.subscribe<keyboard::Key>("keyboard/keyup", 10,
      boost::bind(&Player::keyCallback, this, _1, false));
}

void Player::keyCallback(const keyboard::Key::ConstPtr& msg,
    const bool down_not_up)
{
  const int key = msg->code;
  if (key == keyboard::Key::KEY_a)
  {
    left_input_ = down_not_up;
  }
  if (key == keyboard::Key::KEY_d)
  {
    right_input_ = down_not_up;
  }
  if (key == keyboard::Key::KEY_w)
  {
    jump_input_ = down_not_up;
  }
  // ROS_INFO_STREAM(key << " " << down_not_up << " " << left_input_ << " "
  //     << right_input_ << " " << jump_input_);
}

void Player::update(const cv::Mat mask)
{
  float left_right_move_amount = 3;
  if (left_input_)
  {
    walk(-left_right_move_amount);
  }
  if (right_input_)
  {
    walk(left_right_move_amount);
  }
  if (jump_input_)
  {
    jump();
  }

  cv::Point2f gravity = cv::Point(0, -1.6);
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

void Player::accel(const cv::Point2f dxy)
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
    accel(cv::Point(0, 19));
  }
}

void Player::move(const cv::Point2f dxy)
{
  pos_ += dxy;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bullet_smash");

  cv::Mat background(cv::Size(1000, 700), CV_8UC3, cv::Scalar(255, 255, 105));

  Player player(cv::Point(16, 16));

  std::string mask_file;
  if (!ros::param::get("~level_mask", mask_file))
  {
    ROS_ERROR_STREAM("no level mask file");
    return -1;
  }
  ROS_INFO_STREAM("level mask " << mask_file);
  cv::Mat mask = cv::imread(mask_file, CV_LOAD_IMAGE_GRAYSCALE);
  if (mask.empty())
  {
    ROS_ERROR_STREAM("could not load level mask file");
    return -1;
  }

  // TODO(lwalter) make a level image to load and display instead of the mask
  cv::Mat mask_bgr;
  cv::cvtColor(mask, mask_bgr, CV_GRAY2BGR);
  mask_bgr.copyTo(background, mask);

  ros::Rate rate(10);

  while (ros::ok())
  {
    cv::Mat screen = background.clone();
    player.update(mask);
    player.draw(screen);

    cv::imshow("bullet_smash", screen);
    cv::waitKey(5);

    ros::spinOnce();
    rate.sleep();
  }
}

