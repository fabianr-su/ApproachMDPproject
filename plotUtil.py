import matplotlib.pyplot as plt

## Plot results based on computed policy
def plotTrajectory(mdpInstance, vi, csv, state, figSize=(16, 8)):

    # initialize lists
    alt, altDec, altAcc = [], [], []
    dist, distDec, distAcc = [], [], []
    retractionA, retractionD, extensionD, extensionA = [], [], [], []
    EAS, TAS, velDist = [], [], []
    fuelUsed = 0

    # define initial state
    if not state in vi.pi:
        print('No policy found for this initial state! Looking in the vicinity...')
        i = 0
        oldState = state
        while not state in vi.pi:
            i += 1        
            # try different altitude
            if vi.pi.get((oldState[0]+i, ) + oldState[1:]):
                state = (oldState[0]+i, ) + oldState[1:]
                break
            if vi.pi.get((oldState[0]-i, ) + oldState[1:]):
                state = (oldState[0]-i, ) + oldState[1:]
                break
            # try different speed
            if vi.pi.get((oldState[0], oldState[1]+i) + oldState[2:]):
                state = (oldState[0], oldState[1]+i) + oldState[2:]
                break
            if vi.pi.get((oldState[0], oldState[1]-i) + oldState[2:]):
                state = (oldState[0], oldState[1]-i) + oldState[2:]
            if i == max(oldState[:2]):
                print('Could not find similar initial state. Stopping.')
                return None
        print('Starting from speed: {} m/s, altitude: {} m, config: {}, dist: {} km'.format(state[1], state[0], state[2], state[3]/1000))
        
    print('Computing optimal approach...')    
    # see what will be the first action
    action = vi.pi.get(state)

    while state[3] > 0:
        # save prev. action
        prevAction = action
        # get taken action
        action = vi.pi.get(state)

        # get EAS, TAS
        EAS.append(state[1])
        TAS.append(mdpInstance.env.getRho(0) / mdpInstance.env.getRho(state[0]) * state[1])
        velDist.append(state[3] / 1000)

        # get altitude
        if action == 'extend':
            extensionD.append(state[3] / 1000)
            extensionA.append(state[0])
        elif action == 'retract':
            retractionD.append(state[3] / 1000)
            retractionA.append(state[0])

        if action == 'decel':
            distDec.append(state[3] / 1000)
            altDec.append(state[0])
        elif action == 'accel':
            distAcc.append(state[3] / 1000)
            altAcc.append(state[0])
        else:
            dist.append(state[3] / 1000)
            alt.append(state[0])
        
        # ensure correct plotting if change of action
        if prevAction is not action:
            print('Change in action noticed from {} to {} at distance {}'.format(prevAction, action, state[3]))
            if prevAction == 'decel':
                distDec.append(state[3] / 1000)
                altDec.append(state[0])
            elif prevAction == 'accel':
                distAcc.append(state[3] / 1000)
                altAcc.append(state[0])
            else:
                dist.append(state[3] / 1000)
                alt.append(state[0])

        # time step
        state, prob, reward = mdpInstance.succAndProbReward(state, action)[0]
        fuelUsed -= reward

    # add last state to respective list
    # get EAS, TAS
    EAS.append(state[1])
    TAS.append(mdpInstance.env.getRho(0) / mdpInstance.env.getRho(state[0]) * state[1])
    velDist.append(state[3] / 1000)
    if action == 'extend':
        extensionD.append(state[3] / 1000)
        extensionA.append(state[0])
    elif action == 'retract':
        retractionD.append(state[3] / 1000)
        retractionA.append(state[0])
    elif action == 'decel':
        distDec.append(state[3] / 1000)
        altDec.append(state[0])
    elif action == 'accel':
        distAcc.append(state[3] / 1000)
        altAcc.append(state[0])
    else:
        dist.append(state[3] / 1000)
        alt.append(state[0])

    
    ## plot vertical profiles
    try:
        print('Plotting reference trajectories...')
        csv.plotVerticalProfiles(fs=18, fSize=figSize)
    except:
        print('Failed to plot reference trajectories.')
        plt.figure(figsize=figSize)

    ax = plt.gca()
    # plot MDP profile
    dd, lw, ms = 12, 6, 9
    for (d, a, c, l) in [(dist, alt, 'k', 'Constant speed'), (distDec, altDec, 'r', 'Deceleration'), (distAcc, altAcc, 'y', 'Acceleration'), \
                      (extensionD, extensionA, 'go', 'Flap extension'), (retractionD, retractionA, 'ko', 'Flap retraction')]:
        if d: plt.plot([di+dd for di in d], a, c, linewidth=lw, markersize=ms, label=l)

    # plt.plot([d+dd for d in dist], alt, 'k', linewidth=lw, label='Constant speed')
    # plt.plot([d+dd for d in distDec], altDec, 'r', linewidth=lw, label='Deceleration')
    # plt.plot([d+dd for d in distAcc], altAcc, 'y', linewidth=lw, label='Acceleration')
    # plt.plot([d+dd for d in extensionD], extensionA, 'go', markersize=ms, label='Flap Extension')
    # plt.plot([d+dd for d in retractionD], retractionA, 'yo', markersize=ms, label='Flap Retraction')
    # create legend
    handles, labels = ax.get_legend_handles_labels()

    plt.legend(handles, labels, fontsize=16)
    ## plot velocity profiles
    try:
        csv.plotVelocityProfiles(fs=18, fSize=figSize)
    except:
        plt.figure(figsize=figSize)
    ax = plt.gca()
    # plot MDP profile
    plt.plot([d+dd for d in velDist], TAS, 'k', linewidth=lw)

    print('Fuel used by the aircraft: {} kg'.format(fuelUsed))
    print('Final configuration: speed: {}, altitude: {}, config: {}, dist: {}'.format(state[1], state[0], state[2], state[3]))