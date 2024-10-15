using System;
using UnityEngine;
namespace TCS.CarController {
    [RequireComponent(typeof(Rigidbody))]
    public class CarController : MonoBehaviour {
        [Header("Car Presets")]
        [SerializeField] CarDriveType m_carDriveType = CarDriveType.AllWheelDrive;
        public SpeedType m_speedType = SpeedType.Mph;

        [Header("Car Components"), Space(7)]
        [SerializeField] GameObject[] m_wheelMeshes = new GameObject[4];
        public WheelCollider[] m_wheelColliders = new WheelCollider[4];
        [SerializeField] WheelEffects[] m_wheelEffects = new WheelEffects[4];
        [SerializeField] MeshRenderer m_brakeLightMeshRenderer;
        [SerializeField] MeshRenderer m_reverseLightMeshRenderer;

        [Header("Car Settings"), Space(7)]
        [Tooltip("The amount of offset to apply to the rigidbody center of mass.")]
        [SerializeField] Vector3 m_centerOfMassOffset;
        
        [Tooltip("How far the wheels can turn."), Range(20f, 35f)]
        [SerializeField] float m_maximumSteerAngle;
        
        [Tooltip("How much torque to add to the drive wheels when moving forward.")]
        [SerializeField] float m_fullTorqueOverAllWheels;
        
        [Tooltip("How much torque to add to the drive wheels in reverse.")]
        [SerializeField] float m_reverseTorque;
        
        [Tooltip("How much force should be used for the handbrake.")]
        [SerializeField] float m_maxHandbrakeTorque;
        
        [Tooltip("Will limit how fast the car can go.")]
        [SerializeField] float m_topSpeed = 200.0f;
        
        [Tooltip("The limit of the rev range.")]
        [SerializeField] float m_revRangeBoundary = 1f;
        
        [Tooltip("How much slip until wheel effects start playing."), Range(0.1f, 1f)]
        [SerializeField] float m_slipLimit;
        
        [Tooltip("How much force will be used to apply the brakes")]
        [SerializeField] float m_brakeTorque;
        
        [Tooltip("How quickly digital input reaches the max value.")]
        [SerializeField] float m_smoothInputSpeed = 0.2f;
        
        static int numberOfGears = 5;

        [Header("Steering Helpers"), Space(7)]
        [Tooltip("How much force will be applied to the wheels to prevent flipping. (A good value is around the spring value of the wheel collider.")]
        [SerializeField] float m_antiRollVal = 3500.0f;
        [Tooltip("How much down force to add to the car.")]
        [SerializeField] float m_downForce = 100.0f;
        [Tooltip("0 is pure physics, 1 the car will grip in the direction it's facing.")]
        [SerializeField, Range(0, 1)] float m_steerHelper;
        [Tooltip("0 is no traction control, 1 will try and prevent any slipping")]
        [SerializeField, Range(0, 1)] float m_tractionControl;


        Quaternion[] m_wheelMeshLocalRotations;
        float m_steerAngle;
        int m_gearNum;
        float m_gearFactor;
        float m_oldRotation;
        float m_currentTorque;
        Rigidbody m_rigidbody;
        Vector2 m_currentInputVector;
        Vector2 m_smoothInputVelocity;
        int m_emissionPropertyId;
        float m_currentMaxSteerAngle;

        public bool Skidding { get; private set; }
        public float BrakeInput { get; private set; }
        public float CurrentSteerAngle { get { return m_steerAngle; } }
        public float CurrentSpeed { get { return m_speedType == SpeedType.Mph ? m_rigidbody.linearVelocity.magnitude * 2.23693629f : m_rigidbody.linearVelocity.magnitude * 3.6f; } }
        public float MaxSpeed { get { return m_topSpeed; } }
        public float Revs { get; private set; }
        public float AccelInput { get; private set; }

        void Awake() {
            m_wheelMeshLocalRotations = new Quaternion[4];
            for (var i = 0; i < 4; i++) {
                m_wheelMeshLocalRotations[i] = m_wheelMeshes[i].transform.localRotation;
            }


            m_maxHandbrakeTorque = float.MaxValue;
            m_rigidbody = GetComponent<Rigidbody>();
            m_currentTorque = m_fullTorqueOverAllWheels - (m_tractionControl * m_fullTorqueOverAllWheels);
            m_rigidbody.centerOfMass += m_centerOfMassOffset;
            m_emissionPropertyId = Shader.PropertyToID("_EmissionColor");
        }

        void GearChanging() {
            float f = Mathf.Abs(CurrentSpeed / MaxSpeed);
            float upGearLimit = (1 / (float)numberOfGears) * (m_gearNum + 1);
            float downGearLimit = (1 / (float)numberOfGears) * m_gearNum;

            if (m_gearNum > 0 && f < downGearLimit)
                m_gearNum--;

            if (f > upGearLimit && (m_gearNum < (numberOfGears - 1)))
                m_gearNum++;
        }

        static float CurveFactor(float factor) {
            return 1 - (1 - factor) * (1 - factor);
        }

        static float UnclampedLerp(float from, float to, float value) {
            return (1.0f - value) * from + value * to;
        }

        void CalculateGearFactor() {
            float f = (1 / (float)numberOfGears);

            float targetGearFactor = Mathf.InverseLerp(f * m_gearNum, f * (m_gearNum + 1), Mathf.Abs(CurrentSpeed / MaxSpeed));
            m_gearFactor = Mathf.Lerp(m_gearFactor, targetGearFactor, Time.deltaTime * 5.0f);
        }

        void CalculateRevs() {
            CalculateGearFactor();
            float gearNumFactor = m_gearNum / (float)numberOfGears;
            float revsRangeMin = UnclampedLerp(0f, m_revRangeBoundary, CurveFactor(gearNumFactor));
            float revsRangeMax = UnclampedLerp(m_revRangeBoundary, 1f, gearNumFactor);
            Revs = UnclampedLerp(revsRangeMin, revsRangeMax, m_gearFactor);
        }

        public void Move(float steering, float accel, float footBrake, float handBrake) {
            var input = new Vector2(steering, accel);
            m_currentInputVector = Vector2.SmoothDamp(m_currentInputVector, input, ref m_smoothInputVelocity, m_smoothInputSpeed);
            accel = m_currentInputVector.y;
            steering = m_currentInputVector.x;

            for (var i = 0; i < 4; i++) {
                m_wheelColliders[i].GetWorldPose(out var position, out var quat);
                m_wheelMeshes[i].transform.SetPositionAndRotation(position, quat);
            }

            steering = Mathf.Clamp(steering, -1, 1);
            AccelInput = accel = Mathf.Clamp(accel, 0, 1);
            BrakeInput = footBrake = -1 * Mathf.Clamp(footBrake, -1, 0);
            handBrake = Mathf.Clamp(handBrake, 0, 1);

            m_steerAngle = steering * m_currentMaxSteerAngle;
            m_wheelColliders[0].steerAngle = m_steerAngle;
            m_wheelColliders[1].steerAngle = m_steerAngle;

            SteerHelper();
            ApplyDrive(accel, footBrake);
            CapSpeed();

            if (handBrake > 0f) {
                float handBrakeTorque = handBrake * m_maxHandbrakeTorque;
                m_wheelColliders[2].brakeTorque = handBrakeTorque;
                m_wheelColliders[3].brakeTorque = handBrakeTorque;
                TurnBrakeLightsOn();
            }
            else {
                m_wheelColliders[2].brakeTorque = 0f;
                m_wheelColliders[3].brakeTorque = 0f;
            }

            CalculateRevs();
            GearChanging();
            AddDownForce();
            CheckForWheelSpin();
            TractionControl();
            AntiRoll();
            SetSteerAngle();
        }

        void CapSpeed() {
            float speed = m_rigidbody.linearVelocity.magnitude;
            switch (m_speedType) {
                case SpeedType.Mph:
                    speed *= 2.23693629f;
                    if (speed > m_topSpeed)
                        m_rigidbody.linearVelocity = (m_topSpeed / 2.23693629f) * m_rigidbody.linearVelocity.normalized;
                    break;

                case SpeedType.Kph:
                    speed *= 3.6f;
                    if (speed > m_topSpeed)
                        m_rigidbody.linearVelocity = (m_topSpeed / 3.6f) * m_rigidbody.linearVelocity.normalized;
                    break;
                default:
                    throw new ArgumentOutOfRangeException();
            }
        }

        void ApplyDrive(float accel, float footBrake) {
            float thrustTorque;
            switch (m_carDriveType) {
                case CarDriveType.FrontWheelDrive:
                    thrustTorque = accel * (m_currentTorque / 2f);
                    m_wheelColliders[0].motorTorque = m_wheelColliders[1].motorTorque = thrustTorque;
                    break;
                case CarDriveType.RearWheelDrive:
                    thrustTorque = accel * (m_currentTorque / 2f);
                    m_wheelColliders[2].motorTorque = m_wheelColliders[3].motorTorque = thrustTorque;
                    break;
                case CarDriveType.AllWheelDrive:
                    thrustTorque = accel * (m_currentTorque / 4f);
                    for (var i = 0; i < 4; i++) {
                        m_wheelColliders[i].motorTorque = thrustTorque;
                    }

                    break;
                default:
                    throw new ArgumentOutOfRangeException();
            }

            for (var i = 0; i < 4; i++) {
                if (CurrentSpeed > 5 && Vector3.Angle(transform.forward, m_rigidbody.linearVelocity) < 50f) {
                    m_wheelColliders[i].brakeTorque = m_brakeTorque * footBrake;
                }
                else if (footBrake > 0) {
                    m_wheelColliders[i].brakeTorque = 0f;
                    m_wheelColliders[i].motorTorque = -m_reverseTorque * footBrake;
                }
            }

            if (footBrake > 0) {
                if (CurrentSpeed > 5 && Vector3.Angle(transform.forward, m_rigidbody.linearVelocity) < 50f) {
                    TurnBrakeLightsOn();
                }
                else {
                    TurnBrakeLightsOff();
                    TurnReverseLightsOn();
                }
            }
            else {
                TurnBrakeLightsOff();
                TurnReverseLightsOff();
            }

        }

        void SteerHelper() {
            for (var i = 0; i < 4; i++) {
                m_wheelColliders[i].GetGroundHit(out var wheelHit);
                if (wheelHit.normal == Vector3.zero)
                    return;
            }

            if (Mathf.Abs(m_oldRotation - transform.eulerAngles.y) < 10f) {
                float turnAdjust = (transform.eulerAngles.y - m_oldRotation) * m_steerHelper;
                var velRotation = Quaternion.AngleAxis(turnAdjust, Vector3.up);
                m_rigidbody.linearVelocity = velRotation * m_rigidbody.linearVelocity;
            }

            m_oldRotation = transform.eulerAngles.y;
        }

        void AntiRoll() {
            var travelL = 1.0f;
            var travelR = 1.0f;
            bool groundedLf = m_wheelColliders[0].GetGroundHit(out var wheelHit);

            if (groundedLf)
                travelL = (-m_wheelColliders[0].transform.InverseTransformPoint(wheelHit.point).y - m_wheelColliders[0].radius) / m_wheelColliders[0].suspensionDistance;

            bool groundedRf = m_wheelColliders[1].GetGroundHit(out wheelHit);

            if (groundedRf)
                travelR = (-m_wheelColliders[1].transform.InverseTransformPoint(wheelHit.point).y - m_wheelColliders[1].radius) / m_wheelColliders[1].suspensionDistance;

            float antiRollForce = (travelL - travelR) * m_antiRollVal;

            if (groundedLf)
                m_rigidbody.AddForceAtPosition(m_wheelColliders[0].transform.up * -antiRollForce, m_wheelColliders[0].transform.position);

            if (groundedRf)
                m_rigidbody.AddForceAtPosition(m_wheelColliders[1].transform.up * antiRollForce, m_wheelColliders[1].transform.position);

            bool groundedLr = m_wheelColliders[2].GetGroundHit(out wheelHit);

            if (groundedLr)
                travelL = (-m_wheelColliders[2].transform.InverseTransformPoint(wheelHit.point).y - m_wheelColliders[2].radius) / m_wheelColliders[2].suspensionDistance;

            bool groundedRr = m_wheelColliders[3].GetGroundHit(out wheelHit);

            if (groundedRr)
                travelR = (-m_wheelColliders[3].transform.InverseTransformPoint(wheelHit.point).y - m_wheelColliders[3].radius) / m_wheelColliders[3].suspensionDistance;

            antiRollForce = (travelL - travelR) * m_antiRollVal;

            if (groundedLr)
                m_rigidbody.AddForceAtPosition(m_wheelColliders[2].transform.up * -antiRollForce, m_wheelColliders[2].transform.position);

            if (groundedRr)
                m_rigidbody.AddForceAtPosition(m_wheelColliders[3].transform.up * antiRollForce, m_wheelColliders[3].transform.position);
        }

        void AddDownForce() {
            if (m_downForce > 0)
                m_rigidbody.AddForce(m_downForce * m_rigidbody.linearVelocity.magnitude * -transform.up);
        }

        void CheckForWheelSpin() {
            for (var i = 0; i < 4; i++) {
                m_wheelColliders[i].GetGroundHit(out var wheelHit);

                if (Mathf.Abs(wheelHit.forwardSlip) >= m_slipLimit || Mathf.Abs(wheelHit.sidewaysSlip) >= m_slipLimit) {
                    m_wheelEffects[i].EmitTireSmoke();

                    if (!AnySkidSoundPlaying()) {
                        m_wheelEffects[i].PlayAudio();
                    }

                    continue;
                }

                if (m_wheelEffects[i].IsPlayingAudio)
                    m_wheelEffects[i].StopAudio();

                m_wheelEffects[i].EndSkidTrail();
            }
        }

        void TractionControl() {
            WheelHit wheelHit;
            switch (m_carDriveType) {
                case CarDriveType.FrontWheelDrive:
                    m_wheelColliders[0].GetGroundHit(out wheelHit);
                    AdjustTorque(wheelHit.forwardSlip);

                    m_wheelColliders[1].GetGroundHit(out wheelHit);
                    AdjustTorque(wheelHit.forwardSlip);
                    break;
                case CarDriveType.RearWheelDrive:
                    m_wheelColliders[2].GetGroundHit(out wheelHit);
                    AdjustTorque(wheelHit.forwardSlip);

                    m_wheelColliders[3].GetGroundHit(out wheelHit);
                    AdjustTorque(wheelHit.forwardSlip);
                    break;
                case CarDriveType.AllWheelDrive:
                    for (var i = 0; i < 4; i++) {
                        m_wheelColliders[i].GetGroundHit(out wheelHit);
                        AdjustTorque(wheelHit.forwardSlip);
                    }

                    break;
                default:
                    throw new ArgumentOutOfRangeException();
            }
        }

        void AdjustTorque(float forwardSlip) {
            if (forwardSlip >= m_slipLimit && m_currentTorque >= 0) {
                m_currentTorque -= 10 * m_tractionControl;
            }
            else {
                m_currentTorque += 10 * m_tractionControl;
                if (m_currentTorque > m_fullTorqueOverAllWheels) {
                    m_currentTorque = m_fullTorqueOverAllWheels;
                }
            }
        }

        bool AnySkidSoundPlaying() {
            for (var i = 0; i < 4; i++) {
                if (m_wheelEffects[i].IsPlayingAudio) {
                    return true;
                }
            }

            return false;
        }

        void TurnBrakeLightsOn() {
            m_brakeLightMeshRenderer.material.SetColor(m_emissionPropertyId, Color.white);

            if (!m_brakeLightMeshRenderer.material.IsKeywordEnabled("_EMISSION"))
                m_brakeLightMeshRenderer.material.EnableKeyword("_EMISSION");
        }

        void TurnBrakeLightsOff() {
            m_brakeLightMeshRenderer.material.SetColor(m_emissionPropertyId, Color.black);
        }

        void TurnReverseLightsOn() {
            m_reverseLightMeshRenderer.material.SetColor(m_emissionPropertyId, Color.white);

            if (!m_reverseLightMeshRenderer.material.IsKeywordEnabled("_EMISSION"))
                m_reverseLightMeshRenderer.material.EnableKeyword("_EMISSION");
        }

        void TurnReverseLightsOff() {
            m_reverseLightMeshRenderer.material.SetColor(m_emissionPropertyId, Color.black);
        }

        void SetSteerAngle() {
            m_currentMaxSteerAngle = CurrentSpeed switch {
                < 25f => Mathf.MoveTowards(m_currentMaxSteerAngle, m_maximumSteerAngle, 0.5f),
                > 25f and < 60f => Mathf.MoveTowards(m_currentMaxSteerAngle, m_maximumSteerAngle / 1.5f, 0.5f),
                > 60 => Mathf.MoveTowards(m_currentMaxSteerAngle, m_maximumSteerAngle / 2f, 0.5f),
                _ => m_currentMaxSteerAngle
            };
        }
    }
}