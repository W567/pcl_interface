#!/usr/bin/env python
from pcPubBase import pcPubBase

class pcPublisher(pcPubBase):

    def __init__(self):
        super().__init__()

def main():
    pub = pcPublisher()
    pub.execute()

if __name__ == "__main__":
    main()