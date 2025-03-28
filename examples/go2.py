from src.go2interface import Go2Interface

if __name__ == "__main__":
    ChannelFactoryInitialize(0)
    go2robot = Go2Interface(standup=False)
    go2robot.Init()
    go2robot.Start()

    go2robot.launch_mjpc_gui()
    pass