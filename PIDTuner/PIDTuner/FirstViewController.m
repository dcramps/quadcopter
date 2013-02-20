//
//  FirstViewController.m
//  PIDTuner
//
//  Created by dc on 2013-01-06.
//  Copyright (c) 2013 dc. All rights reserved.
//

#import "FirstViewController.h"

@interface FirstViewController ()

@end

@implementation FirstViewController

- (void)viewDidLoad
{
    [super viewDidLoad];
	// Do any additional setup after loading the view, typically from a nib.
    UITapGestureRecognizer *tap = [[UITapGestureRecognizer alloc] initWithTarget:self action:@selector(tapped)];
    [self.view addGestureRecognizer:tap];
    [pText setText:[NSString stringWithFormat:@"%f",[pStepper value]]];
    [iText setText:[NSString stringWithFormat:@"%f",[iStepper value]]];
    [dText setText:[NSString stringWithFormat:@"%f",[dStepper value]]];
}

- (void)didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}

- (void)tapped
{
    [self.view endEditing:YES];
    [pStepper setValue:[[pText text] doubleValue]];
    [iStepper setValue:[[iText text] doubleValue]];
    [dStepper setValue:[[dText text] doubleValue]];
    [self stepperChanged:pStepper];
    [self stepperChanged:iStepper];
    [self stepperChanged:dStepper];
}

- (IBAction)stepperChanged:(UIStepper*)sender
{
    if (sender == pStepper) {
        [pText setText:[NSString stringWithFormat:@"%f",[pStepper value]]];
    } else if (sender == iStepper) {
        [iText setText:[NSString stringWithFormat:@"%f",[iStepper value]]];
    } else if (sender == dStepper) {
        [dText setText:[NSString stringWithFormat:@"%f",[dStepper value]]];
    }
}

- (IBAction)apply:(id)sender
{
    float p = [pStepper value];
    float i = [iStepper value];
    float d = [dStepper value];
    
    
    float vals[3] = {p, i, d};
    [self sendValues:vals];
}

- (IBAction)abort:(id)sender
{
    float p = -11.0;
    float i = -11.0;
    float d = -11.0;
    
    float vals[3] = {p, i, d};
    [self sendValues:vals];
}

- (void)sendValues:(float[])vals
{
    float p = vals[0];
    float i = vals[1];
    float d = vals[2];
    NSMutableData *data = [NSMutableData dataWithCapacity:0];
    [data appendBytes:&p length:sizeof(float)];
    [data appendBytes:&i length:sizeof(float)];
    [data appendBytes:&d length:sizeof(float)];
        
    NSLog(@"Sent data with length %i",[outputStream write:[data bytes] maxLength:[data length]]);
}

- (IBAction)initNetworkCommunication {
    
	CFReadStreamRef readStream;
	CFWriteStreamRef writeStream;
	CFStreamCreatePairWithSocketToHost(NULL, (CFStringRef)@"169.254.1.1", 80, &readStream, &writeStream);
    
	inputStream = (__bridge NSInputStream *)readStream;
	outputStream = (__bridge NSOutputStream *)writeStream;
    [inputStream removeFromRunLoop:[NSRunLoop currentRunLoop] forMode:NSDefaultRunLoopMode];
    [outputStream removeFromRunLoop:[NSRunLoop currentRunLoop] forMode:NSDefaultRunLoopMode];
    [inputStream close];
    [outputStream close];
	[inputStream setDelegate:self];
	[outputStream setDelegate:self];
	[inputStream scheduleInRunLoop:[NSRunLoop currentRunLoop] forMode:NSDefaultRunLoopMode];
	[outputStream scheduleInRunLoop:[NSRunLoop currentRunLoop] forMode:NSDefaultRunLoopMode];
	[inputStream open];
	[outputStream open];
    
    [self performSelectorInBackground:@selector(checkForConnection) withObject:nil];
}

- (void)checkForConnection
{
    while (1) {
        if ([outputStream hasSpaceAvailable]) {
            [[NSOperationQueue mainQueue] addOperationWithBlock:^ {
                [connectButton setStyle:UIBarButtonItemStyleDone];
                [applyButton setEnabled:YES];
                [abortButton setEnabled:YES];
            }];
        } else {
            [[NSOperationQueue mainQueue] addOperationWithBlock:^ {
                [connectButton setStyle:UIBarButtonItemStylePlain];
                [applyButton setEnabled:NO];
                [abortButton setEnabled:NO];
            }];
        }
        usleep(100000);
    }
}

- (void)stream:(NSStream *)theStream handleEvent:(NSStreamEvent)streamEvent {
	switch (streamEvent) {
		case NSStreamEventOpenCompleted:
            if (theStream == inputStream) {
                NSLog(@"Input stream opened");
            } else {
                NSLog(@"Output stream opened");
            }
			break;
		case NSStreamEventHasBytesAvailable:
			if (theStream == inputStream) {
				uint8_t buffer[1024];
				int len;
				while ([inputStream hasBytesAvailable]) {
					len = [inputStream read:buffer maxLength:sizeof(buffer)];
					if (len > 0) {
						NSString *output = [[NSString alloc] initWithBytes:buffer length:len encoding:NSASCIIStringEncoding];
						if (nil != output) {
							NSLog(@"server said: %@", output);
//							[self messageReceived:output];
						}
					}
				}
			}
			break;
		case NSStreamEventEndEncountered:
            [theStream close];
            [theStream removeFromRunLoop:[NSRunLoop currentRunLoop] forMode:NSDefaultRunLoopMode];
            theStream = nil;
			break;
        case NSStreamEventHasSpaceAvailable:
		case NSStreamEventErrorOccurred:
        case NSStreamEventNone:
		default:
            break;
	}
}

@end
