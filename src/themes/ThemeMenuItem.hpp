//----------------------------------------------------------------------------
//	/^M^\ Count Modula Plugin for VCV Rack - ThemeMenuItem
// common menu item structs for handling selecting themes
//  Copyright (C) 2019  Adam Verspaget
//----------------------------------------------------------------------------

// theme selection menu item
struct ThemeMenuItem : MenuItem {
	THEME_MODULE_NAME *module;
	int themeToUse = 0;
	
	void onAction(const event::Action &e) override {
		module->currentTheme = themeToUse;
	}
};

// theme selection menu item
struct DefaultThemeMenuItem : MenuItem {
	THEME_MODULE_NAME *module;
	int themeToUse = 0;
	
	void onAction(const event::Action &e) override {
		// write the setting
		saveDefaultIntegerValue("DefaultTheme", themeToUse);
		
		// might as well set the current theme too
		module->currentTheme = themeToUse;
	}
};

// module theme menu
struct ThemeMenu : MenuItem {
	THEME_MODULE_NAME *module;
	
	Menu *createChildMenu() override {
		Menu *menu = new Menu;

		// add Standard theme menu item
		ThemeMenuItem *standardMenuItem = createMenuItem<ThemeMenuItem>("Silver", CHECKMARK(module->currentTheme == 0));
		standardMenuItem->module = module;
		standardMenuItem->themeToUse = 0;
		menu->addChild(standardMenuItem);
		
		// add Absinthe theme menu item
		ThemeMenuItem *absintheMenuItem = createMenuItem<ThemeMenuItem>("Absinthe", CHECKMARK(module->currentTheme == 2));
		absintheMenuItem->module = module;
		absintheMenuItem->themeToUse = 2;
		menu->addChild(absintheMenuItem);
		
		// add Blue Moon theme menu item
		ThemeMenuItem *bluemoonMenuItem = createMenuItem<ThemeMenuItem>("Blue Moon", CHECKMARK(module->currentTheme == 5));
		bluemoonMenuItem->module = module;
		bluemoonMenuItem->themeToUse = 5;
		menu->addChild(bluemoonMenuItem);
		
		// add Moonlight theme menu item
		ThemeMenuItem *moonlightMenuItem = createMenuItem<ThemeMenuItem>("Moonlight", CHECKMARK(module->currentTheme == 1));
		moonlightMenuItem->module = module;
		moonlightMenuItem->themeToUse = 1;
		menu->addChild(moonlightMenuItem);

		// add Raven theme menu item
		ThemeMenuItem *ravenMenuItem = createMenuItem<ThemeMenuItem>("Raven", CHECKMARK(module->currentTheme == 3));
		ravenMenuItem->module = module;
		ravenMenuItem->themeToUse = 3;
		menu->addChild(ravenMenuItem);	

		// add Sanguine theme menu item
		ThemeMenuItem *sanguineMenuItem = createMenuItem<ThemeMenuItem>("Sanguine", CHECKMARK(module->currentTheme == 4));
		sanguineMenuItem->module = module;
		sanguineMenuItem->themeToUse = 4;
		menu->addChild(sanguineMenuItem);		
		
		// add Trick or Treat menu item
		ThemeMenuItem *trickOrTreatMenuItem = createMenuItem<ThemeMenuItem>("Trick or Treat", CHECKMARK(module->currentTheme == 6));
		trickOrTreatMenuItem->module = module;
		trickOrTreatMenuItem->themeToUse = 6;
		menu->addChild(trickOrTreatMenuItem);		
			
		return menu;	
	}
};

// default theme menu
struct DefaultThemeMenu : MenuItem {
	THEME_MODULE_NAME *module;
	
	Menu *createChildMenu() override {
		Menu *menu = new Menu;
		
		int currentDefault = readDefaultIntegerValue("DefaultTheme");

		// add Standard theme menu item
		DefaultThemeMenuItem *standardMenuItem = createMenuItem<DefaultThemeMenuItem>("Silver", CHECKMARK(currentDefault == 0));
		standardMenuItem->module = module;
		standardMenuItem->themeToUse = 0;
		menu->addChild(standardMenuItem);
		
		// add Absinthe theme menu item
		DefaultThemeMenuItem *absintheMenuItem = createMenuItem<DefaultThemeMenuItem>("Absinthe", CHECKMARK(currentDefault == 2));
		absintheMenuItem->module = module;
		absintheMenuItem->themeToUse = 2;
		menu->addChild(absintheMenuItem);

		// add Blue Moon theme menu item
		DefaultThemeMenuItem *bluemoonMenuItem = createMenuItem<DefaultThemeMenuItem>("Blue Moon", CHECKMARK(currentDefault == 5));
		bluemoonMenuItem->module = module;
		bluemoonMenuItem->themeToUse = 5;
		menu->addChild(bluemoonMenuItem);		
		
		// add Moonlight theme menu item
		DefaultThemeMenuItem *moonlightMenuItem = createMenuItem<DefaultThemeMenuItem>("Moonlight", CHECKMARK(currentDefault == 1));
		moonlightMenuItem->module = module;
		moonlightMenuItem->themeToUse = 1;
		menu->addChild(moonlightMenuItem);

		// add Raven theme menu item
		DefaultThemeMenuItem *ravenMenuItem = createMenuItem<DefaultThemeMenuItem>("Raven", CHECKMARK(currentDefault == 3));
		ravenMenuItem->module = module;
		ravenMenuItem->themeToUse = 3;
		menu->addChild(ravenMenuItem);	

		// add Sanguine theme menu item
		DefaultThemeMenuItem *sanguineMenuItem = createMenuItem<DefaultThemeMenuItem>("Sanguine", CHECKMARK(currentDefault == 4));
		sanguineMenuItem->module = module;
		sanguineMenuItem->themeToUse = 4;
		menu->addChild(sanguineMenuItem);
		
		// add Trick or Treat menu item
		DefaultThemeMenuItem *trickOrTreatMenuItem = createMenuItem<DefaultThemeMenuItem>("Trick or Treat", CHECKMARK(currentDefault == 6));
		trickOrTreatMenuItem->module = module;
		trickOrTreatMenuItem->themeToUse = 6;
		menu->addChild(trickOrTreatMenuItem);			
		
		return menu;	
	}
};