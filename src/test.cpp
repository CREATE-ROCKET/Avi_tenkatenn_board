#include <Arduino.h>
#include "config.h"

// Standalone test. Define LORA_LOOPBACK_TEST_STANDALONE and exclude main.cpp/decode.cpp.
#ifdef LORA_LOOPBACK_TEST_STANDALONE
namespace
{
    constexpr uint32_t WAIT0 = 20000, AUX_TIMEOUT = 10000, MAX_LATE = 20000;
    constexpr size_t APP = RX_FRAME_SIZE, UART_N = RX_FRAME_SIZE + (LORA_APPEND_RSSI ? 1 : 0);
    struct Stats
    {
        uint32_t dt, dv, ce, le, us, ut, at, n, w0, w1, l0, l1, u0, u1;
        uint64_t ws, ls, uu;
    } st{};
    struct Log
    {
        char s[280];
    };
    QueueHandle_t lq;
    SemaphoreHandle_t mx;
    TaskHandle_t txh;
    uint8_t f[UART_N], ri, cmd, seq;
    bool pend, aut, scheduled;
    uint32_t first, last, waitUs = WAIT0, target, afall, arise;
    char input[48];
    uint8_t ii;
    bool due(uint32_t a, uint32_t b) { return (int32_t)(a - b) >= 0; }
    void out(const char *fmt, ...)
    {
        Log x{};
        va_list a;
        va_start(a, fmt);
        vsnprintf(x.s, sizeof(x.s), fmt, a);
        va_end(a);
        xQueueSend(lq, &x, 0);
    }
    uint8_t check()
    {
        uint8_t x = 0;
        for (size_t i = 0; i < APP - 1; i++)
            x ^= f[i];
        return x;
    }
    void metric(uint32_t v, uint32_t &a, uint32_t &b, uint64_t &s)
    {
        if (!st.n || v < a)
            a = v;
        if (!st.n || v > b)
            b = v;
        s += v;
    }
    void show()
    {
        xSemaphoreTake(mx, portMAX_DELAY);
        Stats s = st;
        xSemaphoreGive(mx);
        uint32_t n = s.n;
        out("TEST_STATS,dl_total=%lu,dl_valid=%lu,checksum_errors=%lu,length_errors=%lu,sequence_gaps=NA,ul_scheduled=%lu,ul_sent=%lu,aux_timeouts=%lu,wait_min=%lu,wait_max=%lu,wait_avg=%llu,lateness_min=%lu,lateness_max=%lu,lateness_avg=%llu,uart_min=%lu,uart_max=%lu,uart_avg=%llu,ack=NA", s.dt, s.dv, s.ce, s.le, s.us, s.ut, s.at, n ? s.w0 : 0, n ? s.w1 : 0, n ? s.ws / n : 0, n ? s.l0 : 0, n ? s.l1 : 0, n ? s.ls / n : 0, n ? s.u0 : 0, n ? s.u1 : 0, n ? s.uu / n : 0);
    }
    void finish()
    {
        bool ok = check() == f[APP - 1], p;
        uint32_t w;
        xSemaphoreTake(mx, portMAX_DELAY);
        st.dt++;
        ok ? st.dv++ : st.ce++;
        p = pend || aut;
        w = waitUs;
        if (ok && p && !scheduled)
        {
            target = last + w;
            scheduled = true;
            st.us++;
            xTaskNotifyGive(txh);
        }
        xSemaphoreGive(mx);
        out("TEST,DL,NA,%lu,%lu,%u,%u,%lu,%lu,NA,NA,NA,NA,NA,%lu,%lu", first, last, ok, p, w, ok && p ? last + w : 0, afall, arise);
        if (!(st.dt % 100))
            show();
    }
    void rx(void *)
    {
        bool active = false;
        for (;;)
        {
            while (Serial1.available())
            {
                uint8_t b = Serial1.read();
                uint32_t t = micros();
                if (!active)
                {
                    if (b != HEADER1)
                        continue;
                    active = true;
                    ri = 0;
                    first = t;
                }
                if (ri >= UART_N)
                {
                    xSemaphoreTake(mx, portMAX_DELAY);
                    st.le++;
                    xSemaphoreGive(mx);
                    active = false;
                    ri = 0;
                    continue;
                }
                f[ri++] = b;
                if (ri == UART_N)
                {
                    last = t;
                    finish();
                    active = false;
                    ri = 0;
                }
            }
            taskYIELD();
        }
    }
    void tx(void *)
    {
        for (;;)
        {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            uint32_t goal = target;
            while (!due(micros(), goal))
                taskYIELD();
            uint32_t a = micros();
            while (!digitalRead(aux) && micros() - a < AUX_TIMEOUT)
                taskYIELD();
            if (!digitalRead(aux))
                st.at++;
            uint32_t start = micros(), late = start - goal;
            if (late > MAX_LATE)
            {
                out("TEST,UL_DROP,NA,%lu,%lu,1,1,%lu,%lu,NA,NA,%lu,%u,%u,%lu,%lu", first, last, waitUs, goal, late, seq, cmd, afall, arise);
                scheduled = false;
                continue;
            }
            uint8_t p[] = {CMD_PREFIX_0, CMD_PREFIX_0, CMD_CHNNL, cmd, seq, (uint8_t)(cmd ^ seq)};
            Serial1.write(p, sizeof(p));
            uint32_t end = micros(), aw = start - last, du = end - start;
            xSemaphoreTake(mx, portMAX_DELAY);
            metric(aw, st.w0, st.w1, st.ws);
            metric(late, st.l0, st.l1, st.ls);
            metric(du, st.u0, st.u1, st.uu);
            st.n++;
            st.ut++;
            pend = false;
            scheduled = false;
            xSemaphoreGive(mx);
            out("TEST,UL,NA,%lu,%lu,1,1,%lu,%lu,%lu,%lu,%lu,%u,%u,%lu,%lu", first, last, waitUs, goal, start, end, late, seq, cmd, afall, arise);
            seq++;
        }
    }
    void command()
    {
        unsigned long v;
        if (sscanf(input, "send %lu", &v) == 1 && v < 256)
        {
            cmd = v;
            pend = true;
            out("TEST_INFO,command_pending,%lu", v);
        }
        else if (sscanf(input, "wait %lu", &v) == 1)
        {
            waitUs = v;
            out("TEST_INFO,wait_us,%lu", v);
        }
        else if (!strcmp(input, "auto on"))
        {
            aut = true;
            out("TEST_INFO,auto_on");
        }
        else if (!strcmp(input, "auto off"))
        {
            aut = false;
            out("TEST_INFO,auto_off");
        }
        else if (!strcmp(input, "stats"))
            show();
        else if (!strcmp(input, "reset"))
        {
            xSemaphoreTake(mx, portMAX_DELAY);
            st = {};
            xSemaphoreGive(mx);
            out("TEST_INFO,stats_reset");
        }
        else
            out("TEST_ERROR,use send <0..255>|wait <us>|auto on|auto off|stats|reset");
    }
    void console(void *)
    {
        for (;;)
        {
            while (Serial.available())
            {
                char c = Serial.read();
                if (c == '\r' || c == '\n')
                {
                    if (ii)
                    {
                        input[ii] = 0;
                        command();
                        ii = 0;
                    }
                }
                else if (ii < sizeof(input) - 1)
                    input[ii++] = c;
                else
                    ii = 0;
            }
            vTaskDelay(1);
        }
    }
    void logger(void *)
    {
        Log x;
        for (;;)
            if (xQueueReceive(lq, &x, portMAX_DELAY))
                Serial.println(x.s);
    }
    void IRAM_ATTR auxEdge()
    {
        if (digitalRead(aux))
            arise = micros();
        else
            afall = micros();
    }
}
void setup()
{
    Serial.begin(115200);
    Serial1.begin(115200, SERIAL_8N1, LoRA_RX, LoRA_TX);
    pinMode(aux, INPUT);
    pinMode(m0, OUTPUT);
    pinMode(m1, OUTPUT);
    digitalWrite(m0, LOW);
    digitalWrite(m1, LOW);
    mx = xSemaphoreCreateMutex();
    lq = xQueueCreate(16, sizeof(Log));
    attachInterrupt(digitalPinToInterrupt(aux), auxEdge, CHANGE);
    xTaskCreatePinnedToCore(logger, "test_log", 4096, nullptr, 1, nullptr, 1);
    xTaskCreatePinnedToCore(tx, "test_tx", 4096, nullptr, 4, &txh, 1);
    xTaskCreatePinnedToCore(rx, "test_rx", 4096, nullptr, 3, nullptr, 0);
    xTaskCreatePinnedToCore(console, "test_cmd", 3072, nullptr, 1, nullptr, 0);
    out("TEST,type,dl_seq,dl_first_us,dl_last_us,dl_valid,pending,configured_wait_us,scheduled_ul_us,actual_ul_start_us,actual_ul_end_us,tx_lateness_us,ul_seq,command,aux_fall_us,aux_rise_us");
    out("TEST_INFO,dl_sequence_and_ack_are_NA_in_current_production_format");
}
void loop() { vTaskDelay(portMAX_DELAY); }
#endif
