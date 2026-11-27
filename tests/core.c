#include "audio_wire.h"
#include "barge_in.h"
#include "dialog_window.h"
#include "grid.h"
#include "intent.h"
#include "tts_text.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static void test_grid(void)
{
    grid_t grid = {0};
    cell_pos_t home = {GRID_CENTER, GRID_CENTER};
    cell_pos_t east = grid_offset(home, EAST);
    cell_pos_t goal = grid_offset(east, EAST);
    cell_pos_t next, target;
    grid_set(&grid, home, CELL_FREE);
    grid_observe(&grid, home, EAST, 101, 25, 25);
    assert(grid_get(&grid, east) == CELL_FREE && grid_get(&grid, goal) == CELL_FREE);
    assert(grid_get(&grid, grid_offset(grid_offset(goal, EAST), EAST)) == CELL_BLOCKED);
    assert(grid_get(&grid, grid_offset(home, NORTH)) == CELL_UNKNOWN);
    assert(grid_next_step(&grid, home, &goal, &next, &target));
    assert(grid_equal(next, east) && grid_equal(target, goal));
    grid_observe(&grid, home, EAST, -1, 25, 25);
    assert(grid_get(&grid, east) == CELL_FREE);

    grid_set(&grid, east, CELL_BLOCKED);
    assert(!grid_next_step(&grid, home, &goal, &next, NULL));
    cell_pos_t south = grid_offset(home, SOUTH);
    grid_set(&grid, south, CELL_FREE);
    grid_set(&grid, grid_offset(south, EAST), CELL_FREE);
    grid_set(&grid, grid_offset(goal, SOUTH), CELL_FREE);
    assert(grid_next_step(&grid, home, &goal, &next, NULL) && grid_equal(next, south));
    for (heading_t heading = NORTH; heading <= WEST; ++heading) grid_mark_scanned(&grid, home, heading);
    assert(grid_next_step(&grid, home, NULL, &next, &target));
    assert(grid_equal(next, south) && grid_equal(target, south));
    puts("PASS grid ray, unknown space, obstacle replanning and frontier search");
}

static void test_intent(void)
{
    intent_t left = intent_from_transcript("嗨，乐鑫，请向左转一点。", &wake_options[3]);
    intent_t left_no_pause = intent_from_transcript("嗨乐鑫，请向左转一点。", &wake_options[3]);
    assert(left.type == INTENT_DRIVE && left.action == LEFT);
    assert(left_no_pause.type == INTENT_DRIVE && left_no_pause.action == LEFT);
    assert(intent_from_transcript("别动。", NULL).type == INTENT_STOP);
    assert(intent_from_transcript("嗨小象，向左转一下。", &wake_options[0]).action == LEFT);
    assert(intent_from_transcript("嗨，喵喵，停下。", &wake_options[1]).type == INTENT_STOP);
    assert(intent_from_transcript("Hi, 喵喵，请前进。", &wake_options[1]).action == FORWARD);
    assert(intent_from_transcript("你好，小智，右转。", &wake_options[2]).action == RIGHT);
    assert(intent_from_transcript("小嘉小嘉，前进", &wake_options[8]).action == FORWARD);
    assert(intent_from_transcript("你好星宝儿，停下", &wake_options[11]).type == INTENT_STOP);
    assert(intent_from_transcript("Deep Seek，左转", &wake_options[19]).action == LEFT);
    assert(wake_option_count == 20);
    for (unsigned i = 0; i < wake_option_count; ++i) {
        char speech[100];
        snprintf(speech, sizeof(speech), "%s，右转。", wake_options[i].phrase);
        intent_t motion = intent_from_transcript(speech, &wake_options[i]);
        assert(motion.type == INTENT_DRIVE && motion.action == RIGHT);
    }
    assert(intent_from_transcript("小鸭小鸭，前进", &wake_options[0]).type == INTENT_CHAT);
    assert(intent_from_transcript("不要前进", NULL).type == INTENT_CHAT);
    assert(intent_from_transcript("他说左转", NULL).type == INTENT_CHAT);
    assert(intent_from_transcript("左转，然后前进", NULL).type == INTENT_CHAT);
    assert(intent_from_transcript("\"左转\"", NULL).type == INTENT_CHAT);
    intent_t quieter = intent_from_transcript("你好小智，小声一点。", &wake_options[2]);
    assert(quieter.type == INTENT_VOLUME_CHANGE && quieter.volume == -8);
    intent_t volume = intent_from_transcript("把音量调到百分之三十五。", NULL);
    assert(volume.type == INTENT_VOLUME_SET && volume.volume == 35);
    volume = intent_from_transcript("音量调到30%！", NULL);
    assert(volume.type == INTENT_VOLUME_SET && volume.volume == 30);
    volume = intent_from_transcript("音量调到十％", NULL);
    assert(volume.type == INTENT_VOLUME_SET && volume.volume == 10);
    volume = intent_from_transcript("我说，音量调到2%。", NULL);
    assert(volume.type == INTENT_VOLUME_SET && volume.volume == 2);
    volume = intent_from_transcript("你好小智，我说，帮我把音量调到百分之二。", &wake_options[2]);
    assert(volume.type == INTENT_VOLUME_SET && volume.volume == 2);
    volume = intent_from_transcript("等一下，把音量调到2%。", NULL);
    assert(volume.type == INTENT_VOLUME_SET && volume.volume == 2);
    assert(intent_from_transcript("我说，音量调到2%会怎么样？", NULL).type == INTENT_CHAT);
    assert(intent_from_transcript("静音", NULL).volume == 0);
    assert(intent_from_transcript("你能告诉我怎么调大音量吗", NULL).type == INTENT_CHAT);
    assert(intent_from_transcript("音量调到101%", NULL).type == INTENT_CHAT);
    intent_t slow = intent_from_transcript("走慢一点", NULL);
    assert(slow.type == INTENT_VOICE_PACE && slow.pace == PACE_SLOW);
    intent_t slow_forward = intent_from_transcript("慢一点往前走", NULL);
    assert(slow_forward.type == INTENT_DRIVE && slow_forward.action == FORWARD && slow_forward.pace == PACE_SLOW);
    intent_t fast_back = intent_from_transcript("后退快一点", NULL);
    assert(fast_back.type == INTENT_DRIVE && fast_back.action == BACK && fast_back.pace == PACE_FAST);
    assert(intent_from_transcript("加速右转", NULL).action == RIGHT);
    assert(intent_from_transcript("慢慢左转，然后前进", NULL).type == INTENT_CHAT);
    assert(intent_from_transcript("不要快一点后退", NULL).type == INTENT_CHAT);
    assert(intent_from_transcript("急停", NULL).type == INTENT_MOTOR_LOCK);
    assert(intent_from_transcript("开启电机", NULL).type == INTENT_MOTOR_ENABLE);
    assert(intent_from_transcript("我在你哪个方向？", NULL).type == INTENT_CHAT);
    assert(intent_from_transcript("我离你多远？", NULL).type == INTENT_CHAT);
    assert(intent_from_transcript("开始建图！", NULL).type == INTENT_EXPLORE);
    intent_t marker = intent_from_transcript("标记这里为客厅。", NULL);
    assert(marker.type == INTENT_MARK && !strcmp(marker.argument, "客厅"));
    intent_t route = intent_from_transcript("导航到客厅", NULL);
    assert(route.type == INTENT_GOTO && !strcmp(route.argument, "客厅"));
    char small[5];
    text_copy(small, sizeof(small), "你好");
    assert(!strcmp(small, "你"));
    puts("PASS wake prefix, allowlisted motion, named navigation and UTF-8 output");
}

static void test_speech_direction(void)
{
    assert(speech_direction((unsigned[]){0, 2, 0, 0}) == DIR_UNKNOWN);
    assert(speech_direction((unsigned[]){30, 7, 2, 1}) == DIR_LEFT);
    assert(speech_direction((unsigned[]){0, 1, 1, 6}) == DIR_RIGHT);
    assert(speech_direction((unsigned[]){0, 1, 6, 2}) == DIR_CENTER);
    assert(speech_direction((unsigned[]){0, 4, 1, 4}) == DIR_UNKNOWN);
    puts("PASS direction voting rejects sparse or conflicting positions");
}

static void test_audio(void)
{
    uint8_t wav[44];
    pcm_wav_header(wav, 32000, 16000);
    assert(!memcmp(wav, "RIFF", 4) && !memcmp(wav + 8, "WAVEfmt ", 8));
    assert(wav[22] == 1 && wav[24] == 0x80 && wav[25] == 0x3e && wav[34] == 16);
    assert(!memcmp(wav + 36, "data", 4) && wav[40] == 0 && wav[41] == 0x7d);

    const uint8_t pcm[] = {0, 0, 0xb8, 0x0b, 0x70, 0x17, 0x20, 0xd1, 0, 0, 0, 0};
    int16_t continuous[8] = {0}, fragmented[8] = {0};
    pcm_resampler_t all = {0}, stream = {0};
    assert(pcm_resample_24_to_16(&all, pcm, sizeof(pcm), continuous, 8) == 4);
    assert(continuous[0] == 1000 && continuous[1] == 5000 && continuous[2] == -8000);
    size_t count = 0;
    count += pcm_resample_24_to_16(&stream, pcm, 1, fragmented + count, 8 - count);
    count += pcm_resample_24_to_16(&stream, pcm + 1, 5, fragmented + count, 8 - count);
    count += pcm_resample_24_to_16(&stream, pcm + 6, 6, fragmented + count, 8 - count);
    assert(count == 4 && !memcmp(continuous, fragmented, count * 2));
    assert(pcm_resampler_finish(&stream, fragmented) == 0);
    puts("PASS mono WAV layout and 24 to 16 kHz resampling across odd network reads");

    all = (pcm_resampler_t){0};
    stream = (pcm_resampler_t){0};
    size_t slow_count = pcm_resample_24_to_18(&all, pcm, sizeof(pcm), continuous, 8);
    assert(slow_count == 3);
    assert(continuous[0] == 750 && continuous[1] == 4500 && continuous[2] == -7500);
    slow_count += pcm_resampler_finish_18(&all, continuous + slow_count);
    count = 0;
    for (size_t offset = 0; offset < sizeof(pcm); ++offset) {
        count += pcm_resample_24_to_18(&stream, pcm + offset, 1, fragmented + count, 8 - count);
    }
    count += pcm_resampler_finish_18(&stream, fragmented + count);
    assert(count == 4 && count == slow_count && !memcmp(continuous, fragmented, count * 2));
    uint8_t second[48000] = {0};
    int16_t slowed[18000];
    assert(pcm_resample_24_to_18(&all, second, sizeof(second), slowed, 18000) == 18000);
    puts("PASS slower playback sample count, odd WebSocket boundaries and final samples");
}

static void test_wifi_profiles(void)
{
    wifi_profiles_t saved = {0}, edited = {0};
    assert(wifi_profiles_add(&saved, NULL, "Home", "test-home"));
    assert(wifi_profiles_add(&saved, NULL, "Phone", "test-phone"));
    assert(wifi_profiles_add(&edited, &saved, "Phone", NULL));
    assert(wifi_profiles_add(&edited, &saved, "Home", NULL));
    assert(strcmp(edited.entries[0].password, "test-phone") == 0);
    assert(strcmp(edited.entries[1].password, "test-home") == 0);
    assert(!wifi_profiles_add(&edited, &saved, "Other", NULL));
    assert(!wifi_profiles_add(&edited, NULL, "Phone", "another-pass"));
    assert(!wifi_profiles_add(&edited, NULL, "Other", "short"));
    assert(!wifi_profiles_add(&edited, NULL, "123456789012345678901234567890123", ""));
    assert(wifi_profiles_add(&edited, &saved, "Open", ""));
    assert(wifi_profiles_add(&edited, NULL, "12345678901234567890123456789012", ""));
    assert(wifi_profiles_add(&edited, NULL, "Fifth", "test-fifth"));
    assert(!wifi_profiles_add(&edited, NULL, "Sixth", "test-sixth"));
    puts("PASS Wi-Fi priority edits, credential retention, validation and capacity");
}

static void test_wifi_rotation(void)
{
    wifi_order_t order = {0};
    assert(wifi_order_step(&order, 0, 0, false, false, false) == WIFI_ORDER_WAIT && order.fallback_ap);

    order = (wifi_order_t){0};
    assert(wifi_order_step(&order, 3, 0, false, false, false) == WIFI_ORDER_CONNECT && order.next == 1);
    assert(wifi_order_step(&order, 3, 200, false, false, true) == WIFI_ORDER_CONNECT && order.next == 2);
    assert(wifi_order_step(&order, 3, 800, true, true, false) == WIFI_ORDER_WAIT && !order.fallback_ap);
    assert(wifi_order_step(&order, 3, 900, false, false, true) == WIFI_ORDER_CONNECT && order.next == 1);
    assert(wifi_order_step(&order, 3, 12899, false, false, false) == WIFI_ORDER_WAIT);
    assert(wifi_order_step(&order, 3, 12900, false, false, false) == WIFI_ORDER_RESET_STA);
    assert(wifi_order_step(&order, 3, 12950, false, false, false) == WIFI_ORDER_CONNECT && order.next == 2);
    assert(wifi_order_step(&order, 3, 13500, false, false, true) == WIFI_ORDER_CONNECT && order.next == 3);
    assert(!order.fallback_ap);
    assert(wifi_order_step(&order, 3, 14000, false, false, true) == WIFI_ORDER_WAIT && order.fallback_ap);
    assert(wifi_order_step(&order, 3, 28999, false, false, false) == WIFI_ORDER_WAIT);
    assert(wifi_order_step(&order, 3, 29000, false, false, false) == WIFI_ORDER_CONNECT && order.next == 1);
    assert(wifi_order_step(&order, 3, 29600, true, true, false) == WIFI_ORDER_WAIT && order.fallback_ap);
    assert(wifi_order_step(&order, 3, 35599, true, false, false) == WIFI_ORDER_WAIT && order.fallback_ap);
    assert(wifi_order_step(&order, 3, 35600, true, false, false) == WIFI_ORDER_WAIT && !order.fallback_ap);
    assert(wifi_order_step(&order, 3, 37200, false, true, true) == WIFI_ORDER_CONNECT && order.next == 1);
    puts("PASS ordered Wi-Fi startup, disconnect, timeout, fallback AP and recovery");
}

static void test_dialog_window(void)
{
    for (int round = 0; round < 3; ++round) {
        int64_t opens = (int64_t)round * 20000000 + 250000;
        dialog_window_t window = {.opens_us = opens, .expires_us = opens + DIALOG_WAIT_MS * 1000};
        assert(!dialog_wait_step(&window, opens - 32 * 1000, true, true, 32));
        assert(!dialog_wait_step(&window, opens, false, true, 32));
        assert(!dialog_wait_step(&window, opens + 32000, true, false, 32));
        assert(!dialog_wait_step(&window, opens + 64000, true, true, 32));
        assert(!dialog_wait_step(&window, opens + 96000, true, true, 32));
        assert(dialog_wait_step(&window, opens + 128000, true, true, 32));
        assert(!dialog_wait_step(&window, opens + 160000, true, true, 32));
    }
    dialog_window_t quiet = {.opens_us = 250000, .expires_us = 8250000};
    assert(!dialog_wait_step(&quiet, 8249999, true, false, 32) && quiet.expires_us);
    assert(!dialog_wait_step(&quiet, 8250000, true, true, 32) && !quiet.expires_us);
    assert(intent_from_transcript("不聊了。", NULL).type == INTENT_END_DIALOG);
    assert(intent_from_transcript("不要结束对话。", NULL).type == INTENT_CHAT);
    assert(intent_from_transcript("停下。", NULL).type == INTENT_STOP);
    puts("PASS follow-up windows, echo guard, timeout and ending phrases");
}

static void test_barge_in(void)
{
    barge_in_t barge = {0};
    for (int i = 0; i < 24; ++i) {
        barge_in_reference(&barge, i % 2 ? 0 : 1400, 32);
        assert(!barge_in_step(&barge, i < 4 ? 600 : 2500, true, 530, 32));
        assert(!barge.silence_ms);
    }
    // Near-end speech can still start a confirmation over audible playback.
    for (int i = 0; i < 2; ++i) {
        barge_in_reference(&barge, 1400, 32);
        assert(!barge_in_step(&barge, 4600, true, 530, 32));
    }
    assert(barge.silence_ms);
    barge = (barge_in_t){0};
    for (int i = 0; i < 24; ++i) {
        assert(!barge_in_step(&barge, 460, true, 530, 32));
        assert(!barge.silence_ms);
    }
    // The speaker alone can cause a transient above threshold. Once muted,
    // tail and a single loud tap must not cancel the pending PCM.
    assert(!barge_in_step(&barge, 1300, true, 530, 32));
    assert(!barge_in_step(&barge, 1300, true, 530, 32) && barge.silence_ms);
    for (int i = 0; i < 6; ++i) {
        barge_in_reference(&barge, 600, 32);
        assert(!barge_in_near_mic(&barge));
        assert(!barge_in_step(&barge, 1200, true, 530, 32));
    }
    while (!barge_in_near_mic(&barge)) {
        barge_in_reference(&barge, 0, 32);
        assert(!barge_in_step(&barge, 1200, true, 530, 32));
    }
    assert(!barge_in_step(&barge, 950, true, 530, 32));
    while (barge.silence_ms) assert(!barge_in_step(&barge, 70, false, 530, 32));
    assert(barge.retry_ms > 0);

    // Speech continues after the echo tail and triggers a new recording.
    for (int i = 0; i < 24; ++i) assert(!barge_in_step(&barge, 480, true, 530, 32));
    assert(!barge_in_step(&barge, 1400, true, 530, 32));
    assert(!barge_in_step(&barge, 1400, true, 530, 32) && barge.silence_ms);
    while (!barge_in_near_mic(&barge)) {
        barge_in_reference(&barge, 0, 32);
        assert(!barge_in_step(&barge, 1100, false, 530, 32));
    }
    assert(!barge_in_step(&barge, 950, true, 530, 32));
    assert(!barge_in_step(&barge, 840, true, 530, 32));
    assert(barge_in_step(&barge, 840, true, 530, 32));
    puts("PASS speech interruption, echo-only recovery and impulse rejection");
}

static void test_tts_segments(void)
{
    const char *story = "你好呀。今天我们去看星星";
    size_t first = strlen("你好呀。");
    assert(tts_text_segment(story, first - 1, false) == 0);
    assert(tts_text_segment(story, first, false) == first);
    assert(tts_text_segment(story, strlen(story), false) == first);
    assert(tts_text_segment(story + first, strlen(story + first), false) == 0);
    assert(tts_text_segment(story + first, strlen(story + first), true) == strlen(story + first));

    const char *pause = "我们今天去看海，";
    assert(tts_text_segment(pause, strlen(pause), false) == strlen(pause));
    assert(tts_text_segment("好呀，", strlen("好呀，"), false) == 0);
    const char *distance = "前面还有12.5厘米。";
    assert(tts_text_segment(distance, strlen("前面还有12."), false) == 0);
    assert(tts_text_segment(distance, strlen(distance), false) == strlen(distance));

    const char *long_line = "一二三四五六七八九十一二三四五六七八九十一二三四五";
    size_t hard = strlen(long_line) - strlen("五");
    assert(tts_text_segment(long_line, hard - 1, false) == 0);
    assert(tts_text_segment(long_line, strlen(long_line), false) == hard);
    puts("PASS TTS incremental sentences, pauses, UTF-8 boundaries, decimals and final tail");
}

int main(void)
{
    test_grid();
    test_intent();
    test_speech_direction();
    test_wifi_profiles();
    test_wifi_rotation();
    test_dialog_window();
    test_barge_in();
    test_audio();
    test_tts_segments();
    return 0;
}
