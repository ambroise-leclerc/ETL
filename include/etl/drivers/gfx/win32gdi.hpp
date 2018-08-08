/// @file win32gdi.hpp
/// @date 23/07/2017 23:17:16
/// @author Ambroise Leclerc
/// @brief Windows GDI display driver
//
// Copyright (c) 2017, Ambroise Leclerc
//   All rights reserved.
//
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions are met:
//
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in
//     the documentation and/or other materials provided with the
//     distribution.
//   * Neither the name of the copyright holders nor the names of
//     contributors may be used to endorse or promote products derived
//     from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS'
//  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
//  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
//  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
//  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  POSSIBILITY OF SUCH DAMAGE./*
#pragma once

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <atomic>
#include <array>
#include <bitset>
#include <cstdint>
#include <iostream>
#include <map>
#include <mutex>
#include <string>
#include <thread>


namespace etl {
namespace gfx {



class RGB24 {
public:
    using DataType = uint8_t;

    uint8_t red() const { return redValue; }
    uint8_t green() const { return greenValue; }
    uint8_t blue() const { return blueValue; }
    void set(uint8_t red, uint8_t green, uint8_t blue) { 
        redValue = red;
        greenValue = green;
        blueValue = blue;
    }

    uint8_t redValue, greenValue, blueValue;
};

class RGB15 {
public:
    RGB15(uint8_t red, uint8_t green, uint8_t blue) : value((red & 0xF8) << 7 | (green & 0xF8) << 2 | blue >> 3) {}
    RGB15(uint16_t value) : value(value) {}

    uint8_t red() const { return (value & 0b111110000000000) >> 7; }
    uint8_t green() const { return (value & 0b1111100000) >> 2; }
    uint8_t blue() const { return (value & 0b11111) << 3; }
    
    uint16_t value;
};


template<uint16_t w, uint16_t h, typename RgbColor>
class RGBBitmap {
public:
    static constexpr uint16_t width() { return w; }
    static constexpr uint16_t height() { return h; }

    std::array<RgbColor, w*h> buffer;
};

template<uint16_t w, uint16_t h>
class Bitmask : public std::bitset<w*h> {
public:
	Bitmask(const char* str) : std::bitset<16>(str, 16, ' ', '*') {}
    static constexpr uint16_t width() { return w; }
    static constexpr uint16_t height() { return h; }
};

template<uint16_t w, uint16_t h>
class Gdi: public std::array<uint8_t, w * h * 3> {
public:
    Gdi(HINSTANCE hInstance, int32_t nCmdShow) : active(false) {
        loopThread = std::thread(&Gdi::messageLoop, this, hInstance, nCmdShow);
    }
    Gdi() = delete;
    Gdi(const Gdi&) = delete;
    Gdi(Gdi&&) = delete;
    ~Gdi() {
        DestroyWindow(windowHandle);
        loopThread.join();
        std::lock_guard<std::mutex> lock(gdiWindowMutex);
        gdiWindow.erase(windowHandle);
    }

    bool isActive() const { return active; }
    static constexpr uint16_t width() { return w; }
    static constexpr uint16_t height() { return h; }

    void selectWindow(uint16_t x, uint16_t y, uint16_t width, uint16_t height) {
        windowX = x;
        windowY = y;
        windowWidth = width;
        windowHeight = height;
    }

    void setPixel(uint16_t x, uint16_t y, uint8_t r, uint8_t g, uint8_t b) {
        auto pixel = 3 * (x + y * w);
        (*this)[pixel] = b;
        (*this)[pixel + 1] = g;
        (*this)[pixel + 2] = r;
    }

    template<typename RgbColor>
    void setPixel(uint16_t x, uint16_t y, RgbColor color) {
        setPixel(x, y, color.red(), color.green(), color.blue());
    }

    void fill(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint8_t r, uint8_t g, uint8_t b) {
        for (auto line = y; line < y + height; ++line) {
            for (auto pixel = 3 * (x + line * w); pixel <= 3 * (x + width + line * w); pixel += 3) {
                (*this)[pixel] = b;
                (*this)[pixel + 1] = g;
                (*this)[pixel + 2] = r;
            }
        }
    }

    template<typename RgbColor>
    void fill(uint16_t x, uint16_t y, uint16_t width, uint16_t height, RgbColor color) {
        fill(x, y, width, height, color.red(), color.green(), color.blue());
    }
    
    template<typename RgbColor, typename Bitmask>
    void fill(uint16_t x, uint16_t y, RgbColor foreground/*, RgbColor background*/, const Bitmask& bitmask) {
		uint16_t bitCounter = 0;
		for (auto h = y; h < min(y + bitmask.height(), height()); ++h)
			for (auto w = x; w < min(x + bitmask.width(), width()); ++w) {
				if (bitmask.test(bitCounter))
					setPixel(w, h, foreground);
			//	else
			//		setPixel(h, w, background);
				++bitCounter;
			}
    }
    
private:
    std::atomic<bool> active;
    std::thread loopThread;
    HWND windowHandle;
    static std::map<HWND, Gdi<w, h>*> gdiWindow;
    static std::mutex gdiWindowMutex;
    uint16_t windowX, windowY, windowWidth, windowHeight;

    void messageLoop(HINSTANCE hInstance, int32_t nCmdShow) {
        WNDCLASS wc = { CS_HREDRAW | CS_VREDRAW, messageHandler, 0, 0, hInstance, nullptr, LoadCursor(nullptr, IDC_ARROW),
                        reinterpret_cast<HBRUSH>(COLOR_WINDOW + 1), nullptr, TEXT("ETLGdiWindow") };
        RegisterClass(&wc);
        RECT rect{ 0, 0, width(), height()};
        AdjustWindowRect(&rect, WS_OVERLAPPEDWINDOW, false);
        windowHandle = CreateWindow(TEXT("ETLGdiWindow"), TEXT("ETL"), WS_OVERLAPPEDWINDOW, CW_USEDEFAULT, 0, rect.right - rect.left, rect.bottom - rect.top, 0, 0, hInstance, 0);
        if (windowHandle) {
            {
                std::lock_guard<std::mutex> lock(gdiWindowMutex);
                gdiWindow[windowHandle] = this;
            }
            ShowWindow(windowHandle, nCmdShow);
            UpdateWindow(windowHandle);
            active = true;
            MSG msg;
            while (GetMessage(&msg, nullptr, 0, 0)) {
                TranslateMessage(&msg);
                DispatchMessage(&msg);
            }
            active = false;
        }
    }

    static LRESULT CALLBACK messageHandler(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam) {
        switch (message) {
            case WM_PAINT: {
                PAINTSTRUCT ps;
                HDC hdc = BeginPaint(hWnd, &ps);
                RECT clientRect;
                GetClientRect(hWnd, &clientRect);
                BITMAPINFO bitmapInfo = { { sizeof(BITMAPINFO), width(), height(), 1, 24 } };
                std::lock_guard<std::mutex> lock(gdiWindowMutex);
                StretchDIBits(hdc, clientRect.left, clientRect.top, clientRect.right, clientRect.bottom, 0, 0, width(), height(), gdiWindow[hWnd]->data(), &bitmapInfo, DIB_RGB_COLORS, SRCCOPY);
                EndPaint(hWnd, &ps);
            } break;
            case WM_DESTROY:
                PostQuitMessage(0);
                break;
            default:
                return DefWindowProc(hWnd, message, wParam, lParam);
        }
        return 0;
    }

};

template<uint16_t width, uint16_t height> std::map<HWND, Gdi<width, height>*> Gdi<width, height>::gdiWindow;
template<uint16_t width, uint16_t height> std::mutex Gdi<width, height>::gdiWindowMutex;


} // namespace gfx
} // namespace etl

