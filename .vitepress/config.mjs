import { defineConfig } from 'vitepress'
import { withSidebar } from 'vitepress-sidebar';
import { groupIconMdPlugin, groupIconVitePlugin } from 'vitepress-plugin-group-icons'
import { tabsMarkdownPlugin } from 'vitepress-plugin-tabs'

const vitePressSidebarOptions = {
    documentRootPath: 'src',
    useFolderLinkFromIndexFile: false,
    useFolderTitleFromIndexFile: true,
    useTitleFromFileHeading: true,
    useTitleFromFrontmatter: true,
    collapseDepth: 2,
    sortMenusByFrontmatterOrder: true,
    manualSortFileNameByPriority: ['introduction', 'concept', 'tutorial', 'reference'],
    frontmatterTitleFieldName: 'sidebar',
    capitalizeFirst: false,
    hyphenToSpace: true,
    excludePattern: ['create_node_reference_markdown_output.md', 'create_node_reference_markdown_output_px4.md']
}

const vitePressOptions = {
    lang: 'en-US',
    title: 'AutoAPMS',
    description: 'A versatile software framework for developing robot behaviors using behavior trees fully integrated with ROS 2.',
    base: '/auto-apms-guide/',

    head: [
        ['link', { rel: 'icon', type: 'image/x-icon', href: '/auto-apms-guide/logo/favicon.ico' }],
        ['link', { rel: 'apple-touch-icon', sizes: '57x57', href: '/auto-apms-guide/logo/apple-icon-57x57.png' }],
        ['link', { rel: 'apple-touch-icon', sizes: '60x60', href: '/auto-apms-guide/logo/apple-icon-60x60.png' }],
        ['link', { rel: 'apple-touch-icon', sizes: '72x72', href: '/auto-apms-guide/logo/apple-icon-72x72.png' }],
        ['link', { rel: 'apple-touch-icon', sizes: '76x76', href: '/auto-apms-guide/logo/apple-icon-76x76.png' }],
        ['link', { rel: 'apple-touch-icon', sizes: '114x114', href: '/auto-apms-guide/logo/apple-icon-114x114.png' }],
        ['link', { rel: 'apple-touch-icon', sizes: '120x120', href: '/auto-apms-guide/logo/apple-icon-120x120.png' }],
        ['link', { rel: 'apple-touch-icon', sizes: '144x144', href: '/auto-apms-guide/logo/apple-icon-144x144.png' }],
        ['link', { rel: 'apple-touch-icon', sizes: '152x152', href: '/auto-apms-guide/logo/apple-icon-152x152.png' }],
        ['link', { rel: 'apple-touch-icon', sizes: '180x180', href: '/auto-apms-guide/logo/apple-icon-180x180.png' }],
        ['link', { rel: 'icon', type: 'image/png', sizes: '192x192', href: '/auto-apms-guide/logo/android-icon-192x192.png' }],
        ['link', { rel: 'icon', type: 'image/png', sizes: '32x32', href: '/auto-apms-guide/logo/favicon-32x32.png' }],
        ['link', { rel: 'icon', type: 'image/png', sizes: '96x96', href: '/auto-apms-guide/logo/favicon-96x96.png' }],
        ['link', { rel: 'icon', type: 'image/png', sizes: '16x16', href: '/auto-apms-guide/logo/favicon-16x16.png' }],
        ['link', { rel: 'manifest', href: '/auto-apms-guide/logo/manifest.json' }],
        ['meta', { name: 'msapplication-TileColor', content: '#1ca5bd' }],
        ['meta', { name: 'msapplication-TileImage', content: '/auto-apms-guide/logo/ms-icon-144x144.png' }],
        ['meta', { name: 'theme-color', content: '#1ca5bd' }],
        ['meta', { property: 'og:url', content: 'https://autoapms.github.io/auto-apms-guide/' }],
        ['meta', { property: 'og:type', content: 'website' }],
        ['meta', { property: 'og:locale', content: 'en' }],
        ['meta', { property: 'og:title', content: 'AutoAPMS - Automated Action Planning and Management System' }],
        ['meta', { property: 'og:site_name', content: 'AutoAPMS' }],
        ['meta', { property: 'og:image', content: 'https://autoapms.github.io/auto-apms-guide/logo/logo-og.png' }],
        ['meta', { property: 'og:image:width', content: '1306' }],
        ['meta', { property: 'og:image:height', content: '910' }],
        ['meta', { name: 'twitter:card', content: 'summary_large_image' }],
        ['meta', { property: 'twitter:domain', content: 'autoapms.github.io' }],
        ['meta', { property: 'twitter:url', content: 'https://autoapms.github.io/auto-apms-guide/' }],
        ['meta', { name: 'twitter:title', content: 'AutoAPMS - Automated Action Planning and Management System' }],
        ['meta', { name: 'twitter:image', content: 'https://autoapms.github.io/auto-apms-guide/logo/logo-og.png' }],

        // Google Search Console
        ['meta', { name: 'google-site-verification', content: 'deYaqCwJq_6IydHhpWd_eiMjPjwJJ_yelf0aAgET3Ow' }]
    ],

    srcDir: 'src',
    srcExclude: ['**/readme.md', '**/README.md', '**/TODO.md', '**/create_node_reference_markdown_output.md', '**/create_node_reference_markdown_output_px4.md'],

    cleanUrls: true,

    sitemap: {
        hostname: 'https://autoapms.github.io/auto-apms-guide/'
    },

    appearance: 'dark',

    themeConfig: {
        // https://vitepress.dev/reference/default-theme-config
        siteTitle: 'AutoAPMS',
        externalLinkIcon: false,
        logo: '/logo/favicon-96x96.png',

        search: {
            provider: "local",
        },

        nav: [
            { text: 'User Guide', link: '/introduction/about', activeMatch: '/\\S' },
            { text: 'API Docs', link: 'https://autoapms.github.io/auto-apms' }
        ],

        socialLinks: [
            { icon: 'github', link: 'https://github.com/AutoAPMS/auto-apms' }
        ],

        outline: [2, 3],

        editLink: {
            pattern: 'https://github.com/AutoAPMS/auto-apms-guide/blob/master/src/:path'
        },
        lastUpdated: {
            text: 'Last update',
            formatOptions: {
                dateStyle: 'medium',
                timeStyle: 'short'
            }
        },
        footer: {
            message: 'Released under the Apache-2.0 License.',
            copyright: 'Copyright © 2024-present Robin Müller'
        }
    },

    // Required for plugin https://github.com/yuyinws/vitepress-plugin-group-icons
    markdown: {
        config(md) {
            md.use(groupIconMdPlugin, {
                titleBar: { includeSnippet: true },
            })
            md.use(tabsMarkdownPlugin)
        },
    },
    vite: {
        plugins: [
            groupIconVitePlugin({
                customIcon: {
                    // Visit https://icon-sets.iconify.design/ for available icons
                    '.cpp': 'vscode-icons:file-type-cpp2',
                    '.hpp': 'vscode-icons:file-type-cpp2',
                    'cmakelists.txt': 'vscode-icons:file-type-cmake',
                    '.cmake': 'vscode-icons:file-type-cmake',
                    '.xml': 'vscode-icons:file-type-xml',
                    'terminal': '<svg xmlns="http://www.w3.org/2000/svg" width="24" height="24" viewBox="0 0 24 24"><rect width="24" height="24" fill="none"/><path fill="#05b8cf" d="M20 4H4a2 2 0 0 0-2 2v12a2 2 0 0 0 2 2h16a2 2 0 0 0 2-2V6a2 2 0 0 0-2-2M6.414 15.707L5 14.293L7.293 12L5 9.707l1.414-1.414L10.121 12zM19 16h-7v-2h7z"/></svg>',
                    '.bash': '<svg xmlns="http://www.w3.org/2000/svg" width="24" height="24" viewBox="0 0 24 24"><rect width="24" height="24" fill="none"/><path fill="#05b8cf" d="M20 4H4a2 2 0 0 0-2 2v12a2 2 0 0 0 2 2h16a2 2 0 0 0 2-2V6a2 2 0 0 0-2-2M6.414 15.707L5 14.293L7.293 12L5 9.707l1.414-1.414L10.121 12zM19 16h-7v-2h7z"/></svg>',
                    '.sh': '<svg xmlns="http://www.w3.org/2000/svg" width="24" height="24" viewBox="0 0 24 24"><rect width="24" height="24" fill="none"/><path fill="#05b8cf" d="M20 4H4a2 2 0 0 0-2 2v12a2 2 0 0 0 2 2h16a2 2 0 0 0 2-2V6a2 2 0 0 0-2-2M6.414 15.707L5 14.293L7.293 12L5 9.707l1.414-1.414L10.121 12zM19 16h-7v-2h7z"/></svg>'
                },
            })
        ],
    }
}

// https://vitepress.dev/reference/site-config
export default defineConfig(withSidebar(vitePressOptions, vitePressSidebarOptions))
