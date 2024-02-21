#include <future>
#include <thread>

int main() {
    std::future<void> fut;
    if (fut.valid()) {
        fut.wait();
    }
    return 0;
}
