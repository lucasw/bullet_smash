/**

  g++ bullet_smash.cpp -lopencv_imgproc -lopencv_highgui -lopencv_core && ./a.out
*/

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class Player
{
  cv::Point pos_;
  cv::Size size_;

public:
  Player(const cv::Point start_pos);

  void move(cv::Point dxy);
  void draw(cv::Mat& screen);
};

Player::Player(const cv::Point start_pos) :
    pos_(start_pos),
    size_(16, 16)
{

}

void Player::draw(cv::Mat& screen)
{
  cv::rectangle(screen, pos_, pos_ + cv::Point(size_),
      cv::Scalar(128, 128, 128), CV_FILLED);
}

void Player::move(cv::Point dxy)
{
  pos_ += dxy;
}

int main(int argn, char** argv)
{
  cv::Mat background(cv::Size(1000, 700), CV_8UC3, cv::Scalar(255, 255, 105));

  Player player(cv::Point(16, background.rows - 32));

  bool do_loop = true;
  while (do_loop)
  {
    cv::Mat screen = background.clone();
    player.draw(screen);

    cv::imshow("bullet_smash", screen);

    int key = cv::waitKey(50);
    if (key == 'a')
    {
      player.move(cv::Point(-4, 0));
    }
    if (key == 'd')
    {
      player.move(cv::Point(4, 0));
    }

  }
}

