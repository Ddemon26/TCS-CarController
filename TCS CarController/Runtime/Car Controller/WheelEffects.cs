using System.Collections;
using UnityEngine;
namespace TCS.CarController {
    [RequireComponent(typeof(AudioSource))]
    public class WheelEffects : MonoBehaviour {
        public static Transform SkidTrailsDetachedParent;

        [SerializeField] Transform m_skidTrailPf;
        [SerializeField] ParticleSystem m_skidParticles;
        [SerializeField] float m_skidTrailOffset = 0.1f;
        public bool IsSkidding { get; private set; }
        public bool IsPlayingAudio { get; private set; }

        AudioSource m_audioSource;
        Transform m_skidTrail;
        WheelCollider m_wheelCollider;

        void Start() {
            if (!m_skidParticles) {
                Debug.LogWarning("No smoke particle system found on car", gameObject);
            }
            else {
                m_skidParticles.Stop();
            }

            m_wheelCollider = GetComponent<WheelCollider>();
            m_audioSource = GetComponent<AudioSource>();
            IsPlayingAudio = false;

            if (!SkidTrailsDetachedParent)
                SkidTrailsDetachedParent = new GameObject("SkidTrails - Detached").transform;
        }

        public void EmitTireSmoke() {
            m_skidParticles.transform.position = transform.position - transform.up * m_wheelCollider.radius;
            m_skidParticles.Emit(1);
            if (!IsSkidding)
                StartCoroutine(StartSkidTrail());
        }

        public void PlayAudio() {
            m_audioSource.Play();
            IsPlayingAudio = true;
        }

        public void StopAudio() {
            m_audioSource.Stop();
            IsPlayingAudio = false;
        }

        public IEnumerator StartSkidTrail() {
            IsSkidding = true;
            m_skidTrail = Instantiate(m_skidTrailPf, transform, true);
            while (!m_skidTrail) {
                yield return null;
            }

            m_skidTrail.localPosition = -Vector3.up * (m_wheelCollider.radius + m_skidTrailOffset);
        }

        public void EndSkidTrail() {
            if (!IsSkidding)
                return;
            IsSkidding = false;
            m_skidTrail.parent = SkidTrailsDetachedParent;
            Destroy(m_skidTrail.gameObject, 10f);
        }
    }
}