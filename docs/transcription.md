# Streaming Transcription notes

## Streaming Transcription with raw WhisperCPP

This is the underlying library used by ros_whisper.

```bash
git clone git@github.com:ggerganov/whisper.cpp.git
cd whisper.cpp
bash ./models/download-ggml-model.sh base.en

# build the main example
make
```

For a quick demo, simply run `make base.en`.

Making the `stream` command:

```bash
# Install SDL2 on Linux
sudo apt-get install libsdl2-dev

# Install SDL2 on Mac OS
brew install sdl2

make stream
```

## Examples

### Streaming

```bash
./stream -m ./models/ggml-base.en.bin -c 1 -t 8 --step 500 --length 5000
```

### Diarization (detecting speaker turns)

(need to test more, not currently working for me)

This uses the [Tinydiarize](https://github.com/akashmjn/tinydiarize) extension for Whisper.

```bash
bash ./models/download-ggml-model.sh small.en-tdrz
```

Then use the -trdz flag and `ggml-small.en-tdrz.bin` model where supported:

```bash
./stream -m ./models/ggml-small.en-tdrz.bin -c 1 -t 6 --step 0 --length 30000 -vth 0.6 -tdrz
```

### Transcribe JFK speech

```bash
# transcribe an audio file
./main -f samples/jfk.wav
```

### Stream BBC radio

```bash
./examples/livestream.sh
Usage: ./examples/livestream.sh stream_url [step_s] [model]

  Example:
    ./examples/livestream.sh http://a.files.bbci.co.uk/media/live/manifesto/audio/simulcast/hls/nonuk/sbr_low/ak/bbc_world_service.m3u8 30 base.en

No url specified, using default: http://a.files.bbci.co.uk/media/live/manifesto/audio/simulcast/hls/nonuk/sbr_low/ak/bbc_world_service.m3u8
[+] Transcribing stream with model 'base.en', step_s 30 (press Ctrl+C to stop):

Buffering audio. Please wait...

 and began to speak out and denounce the atrocities that were taking place. Many of the priests supported the students and tried to give them protection when the lives were at stake. And Daniel Ortega never forgave that. In the years that followed, there were arrests, long jail sentences and extrajudicial killings for anyone who expressed opposition. And that included the clergy.
```

### Real time streaming of local audio

You may first need to figure out the ID of your capture device.

On macOS, you can try:

```bash
$ system_profiler -listDataTypes | grep Audio
SPAudioDataType

$ system_profiler SPAudioDataType
Audio:

    Devices:

        MacBook Pro Microphone:

          Input Channels: 1
          Manufacturer: Apple Inc.
          Current SampleRate: 48000
          Transport: Built-in
          Input Source: MacBook Pro Microphone

        MacBook Pro Speakers:

          Default Output Device: Yes
          Default System Output Device: Yes
          Manufacturer: Apple Inc.
          Output Channels: 2
          Current SampleRate: 48000
          Transport: Built-in
          Output Source: MacBook Pro Speakers

        Danger Microphone:

          Input Channels: 1
          Manufacturer: Apple Inc.
          Current SampleRate: 48000
          Transport: Unknown
          Input Source: Default
# etc
```

Note that in our case MacBook Pro Microphone is the first item on the list, so it is channel 0. Let's use that with the `-c` argument to `stream`.

```bash
$ ./stream -m ./models/ggml-large-v3.bin -c 0 -t 8 --step 500 --length 5000

main: processing 8000 samples (step = 0.5 sec / len = 5.0 sec / keep = 0.2 sec), 8 threads, lang = en, task = transcribe, timestamps = 0 ...
main: n_new_line = 9, no_context = 1

[Start speaking]
 Testing testing.
 - I'm gonna put her down with a little peace and quiet. - Sheldon, do not do your father that.
 I can speak to any of you. Maybe I better talk to him.
 Nope, I got this. George, maybe you should come in first. I got this.
 Thank you.
 You want to explain yourself? I'd rather just go to bed than be up in a few hours sitting here.
 - Look, I understand your time. That is no reason. - I'm not just tired. I'm exhausted.
 It hurts. I get up in the morning to do this job. I live life. And I can just be.
 It's not a lot of money. I keep trying harder and harder. It doesn't even make a difference.
 - Don't eat or punish me, let's just get it over with.
 Thank you.
```

If you want to only transcribe when it hears audio, set step to 0 and vth to a volume between 0 and 1. It is the activation threshold to start transcribing:

```bash
$ ./stream -m ./models/ggml-base.en.bin -c 0 -t 6 --step 0 --length 30000 -vth 0.6

main: processing 0 samples (step = 0.0 sec / len = 30.0 sec / keep = 0.0 sec), 6 threads, lang = en, task = transcribe, timestamps = 1 ...
main: using VAD, will transcribe on speech activity

[Start speaking]

### Transcription 0 START | t0 = 0 ms | t1 = 2368 ms

[00:00.000 --> 00:02.000]   I see you later.

### Transcription 0 END

### Transcription 1 START | t0 = 0 ms | t1 = 5915 ms

[00:00.000 --> 00:02.000]   I say a liar.
[00:02.000 --> 00:04.000]   Are you sure you don't want to stay with me?
[00:04.000 --> 00:06.000]   I don't think so. I don't really understand this.

### Transcription 1 END

### Transcription 2 START | t0 = 0 ms | t1 = 14606 ms

[00:00.000 --> 00:02.000]   Hey, I'll see you later.
[00:02.000 --> 00:04.000]   Are you sure you don't want to stay with me?
[00:04.000 --> 00:06.000]   Uh, so I don't really understand this stuff.
[00:06.000 --> 00:09.000]   And so I felt when we watched "Surety Dance" sing in my state.
[00:09.000 --> 00:12.000]   When Patrick's crazy takes his shirt off in here, I'll be back.
[00:12.000 --> 00:14.000]   You have a good one.

### Transcription 2 END
```

## Using the official ROS 2 Whisper packages

```bash
git clone git@github.com:ros-ai/ros2_whisper.git
cd $RAILBOT_WS/src
ln -s path/to/ros2_whisper .
cd $RAILBOT_WS

# reinitialize rebuilt workspace
colcon build --symlink-install --cmake-args -DPython3_FIND_VIRTUALENV=ONLY
. ./install/local_setup.zsh
```

Now launch service:

```bash
ros2 launch whisper_bringup bringup.launch.py n_thread:=4
```

And finally example client in another terminal (don't forget to initialize your virtualenv):

```bash
micromamba activate railbot
. ./install/local_setup.zsh
ros2 run whisper_demos whisper_on_key
```

Try full bringup of railbot with whisper:

```bash
ros2 launch railbot_bringup macos_launch.py
```