#include "audio_wire.h"
#include "grid.h"
#include "intent.h"
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
}

int main(void)
{
    test_grid();
    test_intent();
    test_audio();
    return 0;
}
