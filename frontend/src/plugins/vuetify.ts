/**
 * plugins/vuetify.ts
 *
 * Framework documentation: https://vuetifyjs.com`
 */

// Styles
import '@mdi/font/css/materialdesignicons.css'
import 'vuetify/styles'

// Composables
import { createVuetify, type ThemeDefinition } from 'vuetify'

const workModeTheme: ThemeDefinition = {
  dark: true,
  colors: {
    // background: '#FFFFFF',
    // surface: '#FFE0B2',
    // primary: '#6200EE',
    // 'primary-darken-1': '#3700B3',
    // secondary: '#03DAC6',
    // 'secondary-darken-1': '#018786',
    // error: '#B00020',
    // info: '#2196F3',
    // success: '#4CAF50',
    // warning: '#FB8C00',
  },
}

// https://vuetifyjs.com/en/introduction/why-vuetify/#feature-guides
export default createVuetify({
  theme: {
    defaultTheme: 'dark',
    themes: {
      'work-mode': workModeTheme
    },
  },
})
