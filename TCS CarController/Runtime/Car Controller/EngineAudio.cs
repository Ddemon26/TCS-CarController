using UnityEngine;

namespace TCS.CarController {
    internal class EngineAudio : MonoBehaviour {
        public enum EngineAudioOptions {
            Simple,
            FourChannel,
        }

        [Header("Style")]
        public EngineAudioOptions m_engineSoundStyle = EngineAudioOptions.FourChannel;
        [Header("Clips"), Space(10)]
        public AudioClip m_lowAccelClip;
        public AudioClip m_lowDecelClip;
        public AudioClip m_highAccelClip;
        public AudioClip m_highDecelClip;
        [Header("Sound Settings"), Space(10)]
        public float m_pitchMultiplier = 1f;
        [SerializeField] float m_lowPitchMin = 1f;
        [SerializeField] float m_lowPitchMax = 6f;
        [SerializeField] float m_highPitchMultiplier = 0.25f;
        [SerializeField] float m_maxRolloffDistance = 500f;
        [SerializeField] float m_dopplerLevel = 1f;
        [SerializeField] bool m_useDoppler = true;
        [Header("Main Camera"), Space(10)]
        [SerializeField] Transform m_mainCameraTransform;

        AudioSource m_lowAccel;
        AudioSource m_lowDecel;
        AudioSource m_highAccel;
        AudioSource m_highDecel;
        bool m_soundStarted;
        CarController m_carController;

        void StartSound() {
            m_carController = GetComponent<CarController>();

            m_highAccel = SetupEngineAudioSource(m_highAccelClip);

            if (m_engineSoundStyle == EngineAudioOptions.FourChannel) {
                m_lowAccel = SetupEngineAudioSource(m_lowAccelClip);
                m_lowDecel = SetupEngineAudioSource(m_lowDecelClip);
                m_highDecel = SetupEngineAudioSource(m_highDecelClip);
            }

            m_soundStarted = true;
        }

        void StopSound() {
            foreach (var source in GetComponents<AudioSource>()) {
                Destroy(source);
            }

            m_soundStarted = false;
        }

        void Update() {
            float camDist = (m_mainCameraTransform.position - transform.position).sqrMagnitude;

            if (m_soundStarted && camDist > m_maxRolloffDistance * m_maxRolloffDistance)
                StopSound();

            if (!m_soundStarted && camDist < m_maxRolloffDistance * m_maxRolloffDistance)
                StartSound();

            if (!m_soundStarted) return;
            float pitch = UnclampedLerp(m_lowPitchMin, m_lowPitchMax, m_carController.Revs);
            pitch = Mathf.Min(m_lowPitchMax, pitch);

            if (m_engineSoundStyle == EngineAudioOptions.Simple) {
                m_highAccel.pitch = pitch * m_pitchMultiplier * m_highPitchMultiplier;
                m_highAccel.dopplerLevel = m_useDoppler ? m_dopplerLevel : 0;
                m_highAccel.volume = 1;
            }
            else {
                m_lowAccel.pitch = pitch * m_pitchMultiplier;
                m_lowDecel.pitch = pitch * m_pitchMultiplier;
                m_highAccel.pitch = pitch * m_highPitchMultiplier * m_pitchMultiplier;
                m_highDecel.pitch = pitch * m_highPitchMultiplier * m_pitchMultiplier;

                float accFade = Mathf.Abs(m_carController.AccelInput);
                float decFade = 1 - accFade;

                float highFade = Mathf.InverseLerp(0.2f, 0.8f, m_carController.Revs);
                float lowFade = 1 - highFade;

                highFade = 1 - ((1 - highFade) * (1 - highFade));
                lowFade = 1 - ((1 - lowFade) * (1 - lowFade));
                accFade = 1 - ((1 - accFade) * (1 - accFade));
                decFade = 1 - ((1 - decFade) * (1 - decFade));

                m_lowAccel.volume = lowFade * accFade;
                m_lowDecel.volume = lowFade * decFade;
                m_highAccel.volume = highFade * accFade;
                m_highDecel.volume = highFade * decFade;

                m_highAccel.dopplerLevel = m_useDoppler ? m_dopplerLevel : 0;
                m_lowAccel.dopplerLevel = m_useDoppler ? m_dopplerLevel : 0;
                m_highDecel.dopplerLevel = m_useDoppler ? m_dopplerLevel : 0;
                m_lowDecel.dopplerLevel = m_useDoppler ? m_dopplerLevel : 0;
            }
        }

        AudioSource SetupEngineAudioSource(AudioClip clip) {
            var source = gameObject.AddComponent<AudioSource>();
            source.clip = clip;
            source.volume = 0;
            source.loop = true;
            source.time = Random.Range(0f, clip.length);
            source.Play();
            source.minDistance = 5f;
            source.maxDistance = m_maxRolloffDistance;
            source.dopplerLevel = 0;
            return source;
        }

        static float UnclampedLerp(float from, float to, float value) {
            return (1.0f - value) * from + value * to;
        }
    }
}