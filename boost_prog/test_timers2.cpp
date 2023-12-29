#include <iostream>
#include <fstream>
#include <boost/bind.hpp>
#include <boost/ref.hpp>
#include <boost/asio.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/utility/value_init.hpp>

#include <boost/chrono/duration.hpp>
#include <boost/chrono/chrono_io.hpp>
#include <boost/chrono/floor.hpp>
#include <boost/chrono/round.hpp>
#include <boost/chrono/ceil.hpp>
#include <boost/chrono/chrono.hpp>
#include <boost/chrono/process_cpu_clocks.hpp>
#include <boost/chrono/ceil.hpp>
#include <boost/chrono/floor.hpp>
#include <boost/chrono/round.hpp>

namespace asio = boost::asio;
namespace chrono = boost::chrono;

class Game {
    std::vector<std::string> data;
    std::ifstream file;
    boost::initialized<bool> is_loaded;
public:
    void update()
    {
        if (!is_loaded.data()) {
            load_file();
            std::cout << data.back() << std::endl;
        }
    }

    void draw()
    {
    }

private:
    void load_file()
    {
        if (is_loaded.data()) {
            return;
        }

        if (!file.is_open()) {
            file.open("a.txt");
        }

        std::string line;
        if (!std::getline(file, line)) {
            is_loaded.data() = true;
            file.close();
            return;
        }
        data.push_back(line);

        if (file.peek() == EOF) {
            is_loaded.data() = true;
            file.close();
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
