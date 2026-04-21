#/bin/bash

# Script to get text input from the user and publish to a topic for
# consumption by a behavior tree using the UIInputAction bt node

TOPIC_NAME="/ui/generic"
MSG_TYPE="robot_ui_interfaces/msg/Generic"

echo "Press Ctrl+C to exit."

instance=$SRANDOM
prev_input=""

while true; do
    read -p "Enter text to publish: " user_input

    # Use previous input if nothing entered
    if [ -z "$user_input" ]; then
        user_input=$prev_input
    fi

    if [ -n "$user_input" ]; then

        echo "Publishing: $user_input"
        instance=$((instance + 1))
        ros2 topic pub --once "$TOPIC_NAME" "$MSG_TYPE" "{name: \"text_input\", value: \"$user_input\", instance: \"${instance}\" }"

        prev_input=$user_input
    fi
done