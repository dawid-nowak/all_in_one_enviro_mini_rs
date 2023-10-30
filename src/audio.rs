use pa::DeviceIndex;
use pa::PortAudio;
use portaudio as pa;
use std::error::Error;
use std::sync::atomic::AtomicBool;
use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::thread;

const INTERLEAVED: bool = true;
const SAMPLE_RATE: u32 = 44_100;
const CHANNELS: u16 = 1;
pub(crate) const FRAMES: u32 = 1024;

pub struct Audio<F>
where
    F: Fn(Vec<f32>) -> Result<(), Box<dyn Error>> + std::marker::Sync + std::marker::Send,
{
    device_name: String,
    is_started: Arc<AtomicBool>,
    handler: Arc<F>,
}

impl<F> Audio<F>
where
    F: Fn(Vec<f32>) -> Result<(), Box<dyn Error>> + std::marker::Sync + std::marker::Send + 'static,
{
    pub fn new(input_device_name: &str, handler: F) -> Result<Self, Box<dyn Error>>
where {
        Ok(Self {
            device_name: String::from(input_device_name),
            is_started: Arc::new(AtomicBool::new(false)),
            handler: Arc::new(handler),
        })
    }

    pub fn start_audio(&self) -> Result<(), Box<dyn Error>> {
        let input_device_name = &self.device_name;
        if self.is_started.load(Ordering::Relaxed) {
            return Ok(());
        }

        tracing::info!("Starting audio");
        let (pa, settings) = Self::initialize(input_device_name)?;
        let (sender, receiver) = ::std::sync::mpsc::sync_channel(100);

        let still_running = self.is_started.clone();
        // A callback to pass to the non-blocking stream.
        let callback = move |pa::InputStreamCallbackArgs {
                                 buffer,
                                 frames,
                                 time,
                                 ..
                             }| {
            let in_buffer_len = buffer.len();
            let samples = Vec::from(buffer);
            assert!(frames == FRAMES as usize);
            let res = sender.send((time.current, in_buffer_len, samples));
            if res.is_err() {
                tracing::error!("Can't send data {res:?}");
                return pa::Complete;
            };
            if still_running.load(Ordering::Relaxed) {
                pa::Continue
            } else {
                tracing::warn!("Stream completed  {still_running:?}");
                let _ = sender.send((time.current, 0, vec![]));
                pa::Abort
            }
        };

        let thread_pa = pa;
        let thread_settings = settings;
        let thread_is_started = self.is_started.clone();
        let external_handler = self.handler.clone();
        let _handler = Arc::new(thread::spawn(move || {
            'end_of_recording: while let Ok(mut stream) =
                thread_pa.open_non_blocking_stream(thread_settings, callback.clone())
            {
                tracing::info!("Stream opened");
                thread_is_started.store(true, Ordering::Relaxed);
                let Ok(_) = stream.start() else { continue };
                while let Ok(true) = stream.is_active() {
                    while let Ok((_current_time, len, samples)) = receiver.recv() {
                        if len == 0 {
                            tracing::warn!("Stream is dead");
                            break 'end_of_recording;
                        }

                        let res = (external_handler)(samples);
                        if res.is_err() {
                            tracing::warn!("Hnadler throw an error {res:?}");
                            break 'end_of_recording;
                        }
                    }
                    tracing::info!("Stream not active... let's restart");
                    let _ = stream.stop();
                }
            }
            tracing::info!("No stream");

            let res = thread_pa.terminate();
            tracing::warn!("PortAudio terminated {res:?}");
            thread_is_started.store(false, Ordering::Relaxed);
        }));

        Ok(())
    }

    pub fn stop_audio(&self) {
        self.is_started.store(false, Ordering::Relaxed);
    }

    fn initialize(
        device_name: &str,
    ) -> Result<(PortAudio, pa::InputStreamSettings<f32>), Box<dyn Error>> {
        let pa = PortAudio::new()?;
        tracing::info!(
            "PortAudio: version: {} {:?} ",
            pa.version(),
            pa.version_text()?
        );

        let input_dev = Self::find_device(&pa, device_name)?;
        let input_info = pa.device_info(input_dev)?;
        tracing::debug!("Input device info {device_name}: {:#?}", &input_info);

        // Construct the input stream parameters.
        tracing::debug!("Input params ?");
        let latency = input_info.default_low_input_latency;
        let input_params =
            pa::StreamParameters::<f32>::new(input_dev, CHANNELS.into(), INTERLEAVED, latency);
        tracing::debug!("Format supported ?");
        pa.is_input_format_supported(input_params, SAMPLE_RATE.into())?;
        let settings = pa::InputStreamSettings::new(input_params, SAMPLE_RATE.into(), FRAMES);
        Ok((pa, settings))
    }

    fn find_device(pa: &PortAudio, device_name: &str) -> Result<DeviceIndex, Box<dyn Error>> {
        let devices = pa.devices()?;
        let mut input = None;
        for dev in devices {
            let (idx, info) = dev?;
            tracing::debug!("Device name {idx:?} {}", info.name);
            if info.name.contains(device_name) {
                input = Some(idx);
            };
        }

        input.ok_or(format!("No {device_name} input device connected to the enviro board").into())
    }
}
