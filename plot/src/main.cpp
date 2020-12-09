#include <cmath>
#include <csignal>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>

#include <Eigen/Dense>

#include <plot.hpp>

using namespace plot;

static volatile std::sig_atomic_t run = true;

int main() {
    std::signal(SIGINT, [](int) {
        run = false;
    });

    TerminalInfo term;
    term.detect();


    //
    // Create some eigen data
    //

    static constexpr int N = 100;
    Eigen::VectorXf t = Eigen::VectorXf::LinSpaced(N, 0, 1);


    RealCanvas<BrailleCanvas> canvas({ { 0.0f, 1.0f }, { 1.0f, -1.0f } }, Size(60, 15), term);
    auto layout = margin(frame(&canvas, term));

    auto bounds = canvas.bounds();
    auto size = canvas.size();
    auto pixel = canvas.unmap_size({ 1, 1 });

    while (true) {
        canvas.clear();

        for (size_t i=0; i<N; ++i) {
            canvas.dot(palette::royalblue, {t(i), t(i)});
        }

        for (auto const& line: layout)
            std::cout << term.clear_line() << line << '\n';

        std::cout << std::flush;

        if (!run)
            break;

        using namespace std::chrono_literals;
        std::this_thread::sleep_for(40ms);

        if (!run) break;

        std::cout << term.move_up(layout.size().y) << std::flush;
    }

    return 0;
}
