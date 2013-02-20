//
//  DetailViewController.h
//  PID
//
//  Created by dc on 2013-01-06.
//  Copyright (c) 2013 dc. All rights reserved.
//

#import <UIKit/UIKit.h>

@interface DetailViewController : UIViewController <UISplitViewControllerDelegate>

@property (strong, nonatomic) id detailItem;

@property (weak, nonatomic) IBOutlet UILabel *detailDescriptionLabel;
@end
