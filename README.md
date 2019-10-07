# Streamr Agent

An example of Robonomics agent dealing with "foreign" payment token.
Specifically Streamr's [DATACoin](https://etherscan.io/token/0x0cf0ee63788a0849fe5297f3407f701e122cc023) in this case.

To get familiar with Streamr project have a look at the [docs](https://streamr.network/docs/introduction).

## Scenario

The agent waits for a new liability contract. At the moment it receives one the parameters (see below) are read and the publishing is begun.

For a simplicity the agent publishes random numbers but it's not a problem to turn it into something more useful.

A user specifies how long he/she desires to receive the data in seconds and the agent creates a thread and performs the task.

The user pays in proportion to the amount of time.

> Note! The stream ID must be created on a user side

In the end of the task the whole history is stored in the result message which could be read via the link sent by email.

## Objective:

* `/period` - the amount of time in seconds
* `/stream_id` - Stream ID
* `/email` - email of the user

## Build

NixOS way:

```
nix bulid -f release.nix
source result/setup.bash (zsh)
```

ROS common way:

```
mkdir -p ws/src && cd ws/src
git clone https://github.com/vourhey/streamr_agent && cd .. 
catkin_make
source devel/setup.bash (zsh)
```

## Launch

```
roslaunch streamr_agent agent.launch login:="<GMAIL_LOGIN>" email_password:="<GMAIL_APP_PASSWORD>" auth_token:="<STREAMR_API_TOKEN>"
```

## Test

There is the `robonomics/promisee_node.py` file for testing purpose only. Have a look at the content to figure out how it works

