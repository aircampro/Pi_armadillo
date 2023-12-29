#include <iostream>
#include <fstream>
#include <boost/bind.hpp>
#include <boost/ref.hpp>
#include <boost/asio.hpp>
#include <boost/asio/steady_timer.hpp>
#include "continuation.hpp"

namespace asio = boost::asio;
namespace chrono = boost::chrono;

class Game {
    std::vector<std::string> data;
    continuation cont;
public:
    Game()
        : cont(boost::bind(&Game::load_file, this))
    {
    }

    void update()
    {
        if (!cont.is_complete()) {
            cont.resume();
            std::cout << data.back() << std::endl;
        }
    }

    void draw()
    {
    }

private:
    void load_file()
    {
        std::ifstream file("a.txt");
        std::string line;
        while (std::getline(file, line)) {
            data.push_back(line);
            if (file.peek() != EOF) {
                cont.suspend();
            }
        }
    }
};

const chrono::milliseconds  
    timer_duration(static_cast<int>(1.0 / 60.0 * 1000));

void on_timer(Game& game, asio::steady_timer& timer)
{
    game.update();
    game.draw();

    timer.expires_from_now(timer_duration);
    timer.async_wait(
        boost::bind(&on_timer, boost::ref(game), boost::ref(timer)));
}

int main()
{
    Game game;

    asio::io_service io_service;
    asio::steady_timer timer(io_service);

    timer.expires_from_now(timer_duration);
    timer.async_wait(
        boost::bind(&on_timer, boost::ref(game), boost::ref(timer)));

    io_service.run();
}
