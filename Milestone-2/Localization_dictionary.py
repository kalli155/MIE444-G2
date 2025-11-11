#uses the given wall distances to find the location

def name (loc, pattern):
    spot = locations.get(str(loc))
    if isinstance(spot, int):
        place = grids.get(spot, {}).get(str(pattern))
        spot = place
    return spot

locations = {
    '[0, 3, 3, 0]': 16,
    '[3, 3, 0, 0]': 17,
    '[3, 0, 0, 3]': 18,
    '[0, 0, 3, 3]': 19,
    '[0, 2, 1, 1]': 'A2 forward',
    '[2, 1, 1, 0]': 'A2 right',
    '[1, 1, 0, 2]': 'A2 backward',
    '[1, 0, 2, 1]': 'A2 left',
    '[0, 1, 0, 2]': 0,
    '[1, 0, 2, 0]': 1,
    '[0, 2, 0, 1]': 2,
    '[2, 0, 1, 0]': 3,
    '[0, 0, 1, 3]': 'A4 forward',
    '[0, 1, 3, 0]': 'A4 right',
    '[1, 3, 0, 0]': 'A4 backward',
    '[3, 0, 0, 1]': 'A4 left',
    #A5 = NA
    '[0, 0, 3, 0]': 4,
    '[0, 3, 0, 0]': 5,
    '[3, 0, 0, 0]': 6,
    '[0, 0, 0, 3]': 7,
    #A7 = NA
    #A8 dublicate
    
    '[1, 1, 2, 0]': 'B1 forward',
    '[1, 2, 0, 1]': 'B1 right',
    '[2, 0, 1, 1]': 'B1 backward',
    '[0, 1, 1, 2]': 'B1 left',
    '[1, 0, 0, 1]': 'B2 forward',
    '[0, 0, 1, 1]': 'B2 right',
    '[0, 1, 1, 0]': 'B2 backward',
    '[1, 1, 0, 0]': 'B2 left',
    #B3 = NA
    '[1, 4, 0, 0]': 'B4 forward',
    '[4, 0, 0, 1]': 'B4 right',
    '[0, 0, 1, 4]': 'B4 backward',
    '[0, 1, 4, 0]': 'B4 left',
    '[0, 3, 0, 1]': 8,
    '[3, 0, 1, 0]': 9,
    '[0, 1, 0, 3]': 10,
    '[1, 0, 3, 0]': 11,
    '[1, 2, 2, 2]': 'B6 forward',
    '[2, 2, 2, 1]': 'B6 right',
    '[2, 2, 1, 2]': 'B6 backward',
    '[2, 1, 2, 2]': 'B6 left',
    #B7 duplicate
    '[1, 0, 2, 4]': 'B8 forward',
    '[0, 2, 4, 1]': 'B8 right',
    '[2, 4, 1, 0]': 'B8 backward',
    '[4, 1, 0, 2]': 'B8 left',
    
    #C1 dublicate
    #C2 = NA
    '[0, 0, 1, 0]': 'C3 forward',
    '[0, 1, 0, 0]': 'C3 right',
    '[1, 0, 0, 0]': 'C3 backward',
    '[0, 0, 0, 1]': 'C3 left',
    #C4 = NA
    #C5 = NA
    #C6 dublicate
    #C7 = NA
    #C8 dublicate
    
    '[3, 4, 0, 0]': 'D1 forward', #changing 5 to 4 for D1 cause of error adding a dublicate option to A1 also
    '[4, 0, 0, 3]': 'D1 right',
    '[0, 0, 3, 4]': 'D1 backward',
    '[0, 3, 4, 0]': 'D1 left',
    '[0, 4, 0, 1]': 12,
    '[4, 0, 1, 0]': 13,
    '[0, 1, 0, 4]': 14,
    '[1, 0, 4, 0]': 15,
    '[1, 3, 0, 2]': 'D3 forward',
    '[3, 0, 2, 1]': 'D3 right',
    '[0, 2, 1, 3]': 'D3 backward',
    '[2, 1, 3, 0]': 'D3 left',
    '[0, 2, 0, 3]': 'D4 forward',
    '[2, 0, 3, 0]': 'D4 right',
    '[0, 3, 0, 2]': 'D4 backward',
    '[3, 0, 2, 0]': 'D4 left',
    #D5 dublicate
    '[3, 0, 0, 4]': 'D6 forward', #changing 5 to 4 for D6 cause of error adding a dublicate option to A1 also
    '[0, 0, 4, 3]': 'D6 right',
    '[0, 4, 3, 0]': 'D6 backward',
    '[4, 3, 0, 0]': 'D6 left'
    #D7 = NA
    #D8 dublicate
}

grids = {
    0: {
        '[0, 0, 0, 0]': 'A3 forward',
        '[0, 1, 1, 1]': 'C1 right',
        '[1, 1, 1, 0]': 'C6 right'
        #fix for C8
    },
    1: {
        '[0, 0, 0, 0]': 'A3 right',
        '[1, 1, 1, 0]': 'C1 backward',
        '[1, 1, 0, 1]': 'C6 backward'
        #fix for C8
    },
    2: {
        '[0, 0, 0, 0]': 'A3 backward',
        '[1, 1, 0, 1]': 'C1 left',
        '[1, 0, 1, 1]': 'C6 left'
        #fix for C8
    },
    3:{
        '[0, 0, 0, 0]': 'A3 left', 
        '[1, 0, 1, 1]': 'C1 forward',
        '[0, 1, 1, 1]': 'C6 forward'
        #fix for C8
    },
    4:{
        '[0, 1, 1, 0]': 'A6 forward', 
        '[1, 1, 1, 0]': 'A8 forward', 
        '[1, 0, 1, 0]': 'D8 backward'
    },
    5:{
        '[1, 1, 0, 0]': 'A6 right', 
        '[1, 1, 0, 1]': 'A8 right', 
        '[0, 1, 0, 1]': 'D8 left'
    },
    6:{
        '[1, 0, 0, 1]': 'A6 backward',
        '[1, 0, 1, 1]': 'A8 backward', 
        '[1, 0, 1, 0]': 'D8 forward'
    },
    7:{
        '[0, 0, 1, 1]': 'A6 left', 
        '[0, 1, 1, 1]': 'A8 left', 
        '[0, 1, 0, 1]': 'D8 right'
    },
    8:{
        '[0, 0, 0, 1]': 'B5 forward',
        '[0, 1, 1, 0]': 'B7 backward'
    },
    9:{
        '[0, 0, 1, 0]': 'B5 right',
        '[1, 1, 0, 0]': 'B7 left'
    },
    10:{
        '[0, 1, 0, 0]': 'B5 backward',
        '[1, 0, 0, 1]': 'B7 forward'
    },
    11:{
        '[1, 0, 0, 0]': 'B5 left',
        '[0, 0, 1, 1]': 'B7 right'
    },
    12:{
        '[1, 1, 0, 1]': 'D2 forward',
        '[0, 1, 1, 1]': 'D5 backward'
    },
    13:{
        '[1, 0, 1, 1]': 'D2 right',
        '[1, 1, 1, 0]': 'D5 left'
    },
    14:{
        '[0, 1, 1, 1]': 'D2 backward',
        '[1, 1, 0, 1]': 'D5 forward'
    },
    15:{
        '[1, 1, 1, 0]': 'D2 left',
        '[1, 0, 1, 1]': 'D5 right'
    },
    16:{
        '[1, 0, 0, 1]': 'A1 forward',
        '[0, 0, 1, 0]': 'D6 backward'
    },
    17:{
        '[0, 0, 1, 1]': 'A1 right',
        '[0, 1, 0, 0]': 'D6 left'
    },
    18:{
        '[0, 1, 1, 0]': 'A1 backward',
        '[1, 0, 0, 0]': 'D6 forward'
    },
    19:{
        '[1, 1, 0, 0]': 'A1 left',
        '[0, 0, 0, 1]': 'D6 right'
    }
}