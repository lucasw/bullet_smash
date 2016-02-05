/**
  Copyright Lucas Walter 2016
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
  void draw(cv::Mat& screen, const cv::Point pos);
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

bool testCollision(const cv::Point2f pt, const cv::Mat mask)
{
  return mask.at<uchar>(pt.y, pt.x) != 0;
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


  cv::Point2f new_pos = pos_;
  cv::Point2f gravity = cv::Point(0, 1.6);
  vel_ += gravity;
  cv::Point2f vel_x(vel_.x, 0);
  cv::Point2f vel_y(0, vel_.y);

  bool downward_vertical_collision = false;
  if ((vel_.y > 0) &&
      (testCollision(pos_ + vel_y + cv::Point2f(0, size_.height), mask)))
  {
      downward_vertical_collision = true;
  }

  if ((vel_.y < 0) &&
      (testCollision(pos_ + vel_y, mask)))
  {
    vel_.y = 0;
  }

  const int ground = mask.rows - 16;
  if ((new_pos.y > ground) or (downward_vertical_collision))
  {
    on_ground_ = true;
    // new_pos.y = ground;
    vel_.y = 0;
  }
  else
  {
    on_ground_ = false;
  }

  new_pos += vel_;

  if (new_pos.y < 0)
  {
    vel_.y = 0;
    new_pos.y = 0;
  }

  if (new_pos.x < 0)
  {
    vel_.x = 0;
    new_pos.x = 0;
  }
  if (new_pos.x > mask.cols)
  {
    vel_.x = 0;
    new_pos.x = mask.cols;
  }

  // air resistance
  vel_ *= 0.95;


  // ground friction
  // if (on_ground_)
  //  vel_.x *= 0.8;

  // collision detection
  #if 0
  cv::Mat player_mask = mask.clone();
  player_mask = cv::Scalar::all(0);
  // TODO(lucasw) later have to draw a line between pos_ and new pos
  // to make sure the player didn't pass through something entirely
  //draw(player_mask, new_pos);
  cv::Point offset(8, 8);
  cv::Point screen_pos = pos_;
  cv::Point screen_new_pos = new_pos;
  cv::line(player_mask, screen_pos + offset, screen_new_pos + offset, cv::Scalar::all(255), 16);
  cv::Mat overlap = player_mask & mask;
  if (cv::countNonZero(overlap) > 0)
  {
    cv::Rect bounds = cv::boundingRect(overlap);
    if (bounds.y
  }
  #endif

  pos_ = new_pos;
}


void Player::draw(cv::Mat& screen, const cv::Point pos)
{
  cv::Point screen_pos = pos;
  // screen_pos.y = screen.rows - pos_.y;
  cv::rectangle(screen, screen_pos, screen_pos + cv::Point(size_),
      cv::Scalar(128, 128, 128), CV_FILLED);
}


void Player::draw(cv::Mat& screen)
{
  draw(screen, pos_);
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
    accel(cv::Point(0, -19));
  }
}

void Player::move(const cv::Point2f dxy)
{
  pos_ += dxy;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "jump");

  cv::Mat background(cv::Size(1000, 700), CV_8UC3, cv::Scalar(255, 255, 105));

  Player player(cv::Point(16, background.rows - 30));

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

    cv::imshow("jump", screen);
    cv::waitKey(5);

    ros::spinOnce();
    rate.sleep();
  }
}

