//
//  FirstViewController.h
//  PIDTuner
//
//  Created by dc on 2013-01-06.
//  Copyright (c) 2013 dc. All rights reserved.
//

#import <UIKit/UIKit.h>

@interface FirstViewController : UIViewController <NSStreamDelegate>
{
    IBOutlet UIStepper *pStepper;
    IBOutlet UIStepper *iStepper;
    IBOutlet UIStepper *dStepper;
    
    IBOutlet UITextField *pText;
    IBOutlet UITextField *iText;
    IBOutlet UITextField *dText;
    
    IBOutlet UIButton *applyButton;
    IBOutlet UIButton *abortButton;
    
    IBOutlet UIBarButtonItem *connectButton;
    
    NSInputStream *inputStream;
	NSOutputStream *outputStream;
}

- (void)tapped;
- (IBAction)stepperChanged:(id)sender;
- (IBAction)apply:(id)sender;
- (IBAction)abort:(id)sender;
- (void)sendValues:(float[])vals;
- (IBAction)initNetworkCommunication;

@end
