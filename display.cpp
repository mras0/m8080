#include "display.h"

#include <thread>
#include <functional>
#include <mutex>
#include <condition_variable>
#include <stdexcept>
#include <atomic>
#include <cassert>

#define NOMINMAX
#include <windows.h>

const wchar_t* ClassName = L"DisplayWindowClass";

class display::impl {
public:
    explicit impl(int w, int h, int xscale, int yscale)
        : w_(w), h_(h)
        , hwnd_ {nullptr}
        , thread_([this]() { main_loop(); })
    {
        // Hack, but the thread needs to create a message queue before posting a thread message
        while (!thread_alive_)
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (!do_func([&]() { do_create(xscale, yscale); })) {
            throw std::runtime_error { "Error creating window" };
        }
    }

    ~impl() {
        while (do_func([&]() {
            if (hwnd_)
                PostMessage(hwnd_, WM_CLOSE, 0, 0);
            else {
                printf("Warning: Window closed!\n");
                assert(thread_alive_);
                ExitThread(0xDEAD);
            }
        }))
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        thread_.join();
    }

    bool show(const uint32_t* data) {
        return do_func([&]() { do_show(data); });
    }

    std::vector<uint8_t> events()
    {
        std::unique_lock<std::mutex> lock_ { mtx_ };
        std::vector<uint8_t> evts;
        std::swap(evts, events_);
        return evts;
    }

private:
    int             w_;
    int             h_;
    HWND            hwnd_;
    HDC             hdc;
    HBITMAP         hbm;
    std::mutex      mtx_;
    std::vector<uint8_t> events_;
    std::condition_variable cnd_;
    std::function<void(void)> f_;
    std::atomic<bool> thread_alive_ = false;
    std::thread     thread_; // Must be last

    void add_event(uint8_t evt)
    {
        std::unique_lock<std::mutex> lock_ { mtx_ };
        events_.push_back(evt);
    }

    bool do_func(std::function<void(void)> f)
    {
        if (!thread_alive_)
            return false;
        std::unique_lock<std::mutex> lock_ { mtx_ };
        assert(!f_);
        f_ = f;
        PostThreadMessage(GetThreadId(thread_.native_handle()), WM_APP, 0, 0);
        cnd_.wait(lock_, [this]() { return !f_; });
        return hwnd_ != nullptr;
    }

    void do_create(int xscale, int yscale) {
        WNDCLASS wc;
        HINSTANCE hInstance = GetModuleHandle(NULL);
        DWORD dwStyle = WS_OVERLAPPEDWINDOW;
        RECT rect;
        wchar_t tmp[256];

        ZeroMemory(&wc, sizeof(wc));
        wc.style = CS_HREDRAW | CS_VREDRAW;
        wc.lpfnWndProc = DisplayWndProc;
        wc.hInstance = hInstance;
        wc.hIcon = LoadIcon(NULL, IDI_APPLICATION);
        wc.hCursor = LoadCursor(NULL, IDC_ARROW);
        wc.hbrBackground = (HBRUSH)GetStockObject(BLACK_BRUSH);
        wc.lpszClassName = ClassName;

        if (!RegisterClass(&wc)) {
            printf("Error registering class: %d\n", GetLastError());
            std::abort();
        }
        rect.left = 0;
        rect.top = 0;
        rect.right = w_ * xscale;
        rect.bottom = h_ * yscale;
        AdjustWindowRect(&rect, dwStyle, FALSE);

        wsprintf(tmp, L"Display %dx%d", w_, h_);
        hwnd_ = CreateWindow(ClassName, tmp, dwStyle, 0, 0, rect.right - rect.left, rect.bottom - rect.top, NULL, NULL, hInstance, this);
        if (!hwnd_) {
            printf("Error creating window: %d\n", GetLastError());
            std::abort();
        }

        ShowWindow(hwnd_, SW_NORMAL);
        UpdateWindow(hwnd_);
    }

    void main_loop()
    {
        MSG msg;
        thread_alive_ = true;
        while (GetMessage(&msg, nullptr, 0, 0)) {
            if (msg.message == WM_QUIT)
                break;
            if (msg.message == WM_APP) {
                {
                    std::unique_lock<std::mutex> lock_ { mtx_ };
                    assert(f_);
                    f_();
                    f_ = nullptr;
                }
                cnd_.notify_all();
                continue;
            }
            TranslateMessage(&msg);
            DispatchMessage(&msg);
        }
        add_event(event_quit);
        thread_alive_ = false;
    }

    bool do_show(const uint32_t* data) {
        // Update bitmap
        BITMAPINFO bmi;
        ZeroMemory(&bmi, sizeof(bmi));
        bmi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
        bmi.bmiHeader.biWidth = w_;
        bmi.bmiHeader.biHeight = -h_;
        bmi.bmiHeader.biPlanes = 1;
        bmi.bmiHeader.biCompression = BI_RGB;
        bmi.bmiHeader.biBitCount = 32;
        SetDIBits(hdc, hbm, 0, h_, data, &bmi, DIB_RGB_COLORS);
        // Repaint window
        RedrawWindow(hwnd_, nullptr, nullptr, RDW_INVALIDATE | RDW_UPDATENOW);
        return true;
    }


    static LRESULT CALLBACK DisplayWndProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam) {
        impl& self = *reinterpret_cast<impl*>(GetWindowLongPtr(hwnd, GWLP_USERDATA));
        switch (uMsg) {
            case WM_NCCREATE:
                SetWindowLongPtr(hwnd, GWLP_USERDATA, (LONG_PTR)(((CREATESTRUCT*)lParam)->lpCreateParams));
                break;
            case WM_CREATE:
                {
                    HDC hdc = GetWindowDC(hwnd);
                    self.hdc = CreateCompatibleDC(hdc);
                    self.hbm = CreateCompatibleBitmap(hdc, self.w_, self.h_);
                    SelectObject(self.hdc, self.hbm);
                    ReleaseDC(hwnd, hdc);
                    break;
                }
            case WM_DESTROY:
                DeleteObject(self.hbm);
                DeleteDC(self.hdc);
                PostQuitMessage(0);
                break;
            case WM_KEYUP:
            case WM_KEYDOWN:
                switch (wParam) {
                case VK_ESCAPE:
                case VK_LEFT:
                case VK_UP:
                case VK_RIGHT:
                case VK_DOWN:
                case '1':
                case '2':
                case 'C':
                case VK_SPACE:
                    self.add_event(static_cast<uint8_t>((uMsg == WM_KEYDOWN ? 0x00 : event_keyup_mask) | wParam));
                    break;
                }
                break;
            case WM_ERASEBKGND:
                return TRUE;
            case WM_PAINT:
                {
                    PAINTSTRUCT ps;
                    if (BeginPaint(hwnd, &ps) && !IsRectEmpty(&ps.rcPaint)) {
                        //StretchBlt(ps.hdc, 0, 0, info->w * xscale(info->mode), info->h * yscale(info->mode), info->hdc, 0, 0, info->w, info->h, SRCCOPY);
                        RECT rcClient;
                        GetClientRect(hwnd, &rcClient);
                        StretchBlt(ps.hdc, 0, 0, rcClient.right-rcClient.left, rcClient.bottom-rcClient.top, self.hdc, 0, 0, self.w_, self.h_, SRCCOPY);
                        EndPaint(hwnd, &ps);
                    }
                    return 0;
                }
        }
        return DefWindowProc(hwnd, uMsg, wParam, lParam);
    }
};

display::display(int w, int h, int xscale, int yscale) : impl_(new impl{w, h, xscale, yscale}) {
}

display::~display() = default;

bool display::show(const uint32_t* data) {
    return impl_->show(data);
}

std::vector<uint8_t> display::events()
{
    return impl_->events();
}
