# SROS2 deployment material

Do not store a production keystore or private key in this repository. Create it on a controlled provisioning host, generate one enclave per trust boundary, review the generated policy, sign it with the deployment CA, and provision only the required enclave artifacts to each Jetson/ground station.

Minimum trust boundaries:

- `/air_ground/sensors`
- `/air_ground/planning`
- `/air_ground/control_mux`
- `/air_ground/system_supervisor`
- `/air_ground/web_gateway`
- `/air_ground/recording`

Start with the official SROS2 workflow:

```bash
ros2 security create_keystore /secure/provisioning/air_ground_keystore
ros2 security create_enclave /secure/provisioning/air_ground_keystore /air_ground/system_supervisor
ros2 security create_enclave /secure/provisioning/air_ground_keystore /air_ground/control_mux
ros2 security create_enclave /secure/provisioning/air_ground_keystore /air_ground/web_gateway
```

Generate policy from an isolated commissioning run, remove unintended topic/service access, then regenerate signed artifacts. Deploy with `ROS_SECURITY_STRATEGY=Enforce`; never use permissive fallback in production.

